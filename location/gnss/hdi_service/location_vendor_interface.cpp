/*
 * Copyright (c) 2022 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "location_vendor_interface.h"
#include <array>
#include <cerrno>
#include <condition_variable>
#include <dlfcn.h>
#include <memory>
#include <pthread.h>
#include <string>
#include <utility>
#include "gnss_hdf_log.h"

namespace OHOS::HDI::Location {
namespace {
struct Session {
    void* handle = nullptr;
    GnssVendorInterface methods{};
    GnssCallbackStruct callbacks{};
    GnssCallbackStruct vendorCallbacks{};
    size_t calls = 0;
    size_t callbacksInFlight = 0;
    bool enabled = false;
    bool transition = false;
    bool borrowedModule = false;
};
std::mutex gate;
std::condition_variable changed;
std::shared_ptr<Session> current;
std::shared_ptr<Session> retiring;
bool loading = false;
bool retireFailed = false;
bool reaperStarted = false;
pthread_t reaper{};
thread_local unsigned callDepth = 0;
thread_local unsigned callbackDepth = 0;

class Call {
public:
    Call() {
        std::lock_guard<std::mutex> lock(gate);
        session = current;
        if (session) { ++session->calls; ++callDepth; }
    }
    ~Call() {
        if (!session) return;
        std::lock_guard<std::mutex> lock(gate);
        --session->calls; --callDepth; changed.notify_all();
    }
    std::shared_ptr<Session> session;
};
template<class Member, class... Args> void Forward(Member member, Args... args)
{
    std::shared_ptr<Session> session;
    auto callback = GnssBasicCallbackIfaces{}.*member;
    {
        std::lock_guard<std::mutex> lock(gate);
        session = current;
        if (!session || !session->enabled) return;
        callback = session->callbacks.gnssCallback.*member;
        if (!callback) return;
        ++session->callbacksInFlight; ++callbackDepth;
    }
    callback(args...);
    {
        std::lock_guard<std::mutex> lock(gate);
        --session->callbacksInFlight; --callbackDepth; changed.notify_all();
    }
}
void Position(GnssLocation* value) { Forward(&GnssBasicCallbackIfaces::locationUpdate, value); }
void Status(uint16_t* value) { Forward(&GnssBasicCallbackIfaces::gnssWorkingStatusUpdate, value); }
void Satellites(GnssSatelliteStatus* value) { Forward(&GnssBasicCallbackIfaces::satelliteStatusUpdate, value); }
void Nmea(int64_t time, const char* text, int size) { Forward(&GnssBasicCallbackIfaces::nmeaUpdate,time,text,size); }
void Capabilities(uint32_t value) { Forward(&GnssBasicCallbackIfaces::capabilitiesUpdate,value); }
void Reference(int value) { Forward(&GnssBasicCallbackIfaces::requestRefInfo,value); }
void EphemerisRequest() { Forward(&GnssBasicCallbackIfaces::requestExtendedEphemeris); }
void Cached(const GnssLocation** values, size_t size)
{
    std::shared_ptr<Session> session;
    OnCachedLocationsChange callback;
    {
        std::lock_guard<std::mutex> lock(gate); session=current;
        if (!session || !session->enabled || !session->callbacks.gnssCacheCallback.cachedLocationUpdate) return;
        callback=session->callbacks.gnssCacheCallback.cachedLocationUpdate;
        ++session->callbacksInFlight; ++callbackDepth;
    }
    callback(values,size);
    { std::lock_guard<std::mutex> lock(gate); --session->callbacksInFlight; --callbackDepth; changed.notify_all(); }
}
GnssCallbackStruct Trampolines(const GnssCallbackStruct& client)
{
    GnssCallbackStruct cb{}; cb.size=sizeof(cb); cb.gnssCallback.size=sizeof(cb.gnssCallback);
    cb.gnssCacheCallback.size=sizeof(cb.gnssCacheCallback);
    auto& out=cb.gnssCallback; const auto& in=client.gnssCallback;
    out.locationUpdate=in.locationUpdate?Position:nullptr;
    out.gnssWorkingStatusUpdate=in.gnssWorkingStatusUpdate?Status:nullptr;
    out.satelliteStatusUpdate=in.satelliteStatusUpdate?Satellites:nullptr;
    out.nmeaUpdate=in.nmeaUpdate?Nmea:nullptr;
    out.capabilitiesUpdate=in.capabilitiesUpdate?Capabilities:nullptr;
    out.requestRefInfo=in.requestRefInfo?Reference:nullptr;
    out.requestExtendedEphemeris=in.requestExtendedEphemeris?EphemerisRequest:nullptr;
    cb.gnssCacheCallback.cachedLocationUpdate=client.gnssCacheCallback.cachedLocationUpdate?Cached:nullptr;
    return cb;
}
int Enable(GnssCallbackStruct* callbacks)
{
    if (!callbacks || callbacks->size!=sizeof(*callbacks) ||
        callbacks->gnssCallback.size!=sizeof(callbacks->gnssCallback) || !callbacks->gnssCallback.locationUpdate) return -EINVAL;
    Call call; if (!call.session) return -ENODEV;
    auto session=call.session;
    {
        std::lock_guard<std::mutex> lock(gate);
        if (session->enabled || session->transition) return -EBUSY;
        session->callbacks=*callbacks; session->enabled=true; session->transition=true;
    }
    session->vendorCallbacks=Trampolines(*callbacks);
    int result=session->methods.enableGnss(&session->vendorCallbacks);
    { std::lock_guard<std::mutex> lock(gate); session->transition=false;
      if(result){session->enabled=false; session->callbacks={};} }
    return result;
}
int Disable()
{
    Call call; if (!call.session) return -ENODEV;
    { std::lock_guard<std::mutex> lock(gate);
      if(call.session->transition)return -EBUSY;
      call.session->enabled=false;call.session->transition=true; }
    int result=call.session->methods.disableGnss();
    { std::lock_guard<std::mutex> lock(gate); call.session->callbacks={};call.session->transition=false; }
    return result;
}
int Start(uint32_t type) { Call call; return call.session?call.session->methods.startGnss(type):-ENODEV; }
int Stop(uint32_t type) { Call call; return call.session?call.session->methods.stopGnss(type):-ENODEV; }
int Inject(int type,GnssReferenceInfo* info) { Call call; return call.session?call.session->methods.injectsGnssReferenceInfo(type,info):-ENODEV; }
int Config(GnssConfigParameter* config) { Call call; return call.session?call.session->methods.setGnssConfigPara(config):-ENODEV; }
void Remove(uint16_t flags) { Call call; if(call.session)call.session->methods.removeAuxiliaryData(flags); }
int InjectExtended(char* data,int size) { Call call; return call.session?call.session->methods.injectExtendedEphemeris(data,size):-ENODEV; }
int CacheSize() { Call call; return call.session&&call.session->methods.getCachedLocationsSize?call.session->methods.getCachedLocationsSize():-ENOTSUP; }
void Flush() { Call call; if(call.session&&call.session->methods.flushCachedGnssLocations)call.session->methods.flushCachedGnssLocations(); }
const void* Module(int id)
{
    Call call; if(!call.session)return nullptr;
    const void* result=call.session->methods.getGnssModuleIface(id);
    // This ABI exposes unscoped module pointers: preserve mapping if any are borrowed.
    if(result){std::lock_guard<std::mutex> lock(gate);call.session->borrowedModule=true;}
    return result;
}
const std::array<GnssVendorInterface,4> proxies=[] {
    std::array<GnssVendorInterface,4> result{};
    for(size_t i=0;i<result.size();++i)result[i]={sizeof(GnssVendorInterface),Enable,Disable,Start,Stop,Inject,Config,
        Remove,InjectExtended,(i&1)?CacheSize:nullptr,(i&2)?Flush:nullptr,Module};
    return result;
}();
void* Reap(void*)
{
    std::shared_ptr<Session> session;
    {
        std::unique_lock<std::mutex> lock(gate); session=retiring;
        changed.wait(lock,[&]{return session->calls==0;});
    }
    // Always execute disable on this non-provider thread: its return includes a real worker join.
    int result=session->methods.disableGnss();
    {
        std::unique_lock<std::mutex> lock(gate);
        changed.wait(lock,[&]{return session->callbacksInFlight==0;});
        if(result || session->borrowedModule) {
            retireFailed=true; changed.notify_all();
            return nullptr; // Fail closed: never unmap active code or an unscoped borrowed module.
        }
    }
    int closed=dlclose(session->handle);
    { std::lock_guard<std::mutex> lock(gate);
      if(closed)retireFailed=true;else retiring.reset();changed.notify_all(); }
    return nullptr;
}
void JoinCompleted()
{
    pthread_t thread{};bool join=false;
    {
        std::lock_guard<std::mutex> lock(gate);
        if(reaperStarted && (!retiring || retireFailed)) { thread=reaper; reaperStarted=false;join=true; }
    }
    if(join)pthread_join(thread,nullptr);
}
} // namespace
LocationVendorInterface::LocationVendorInterface() = default;
LocationVendorInterface::~LocationVendorInterface() { CleanUp(); }
LocationVendorInterface* LocationVendorInterface::Singleton()
{
    static LocationVendorInterface singleton; return &singleton;
}
LocationVendorInterface* LocationVendorInterface::GetInstance()
{
    auto* singleton=Singleton(); singleton->Init(); return singleton;
}
void LocationVendorInterface::DestroyInstance()
{
    // A bounded lifetime singleton keeps the proxy address valid for racing/reentrant callers.
    Singleton()->CleanUp();
}
void LocationVendorInterface::Init()
{
    JoinCompleted();
    {
        std::unique_lock<std::mutex> lock(gate);
        changed.wait(lock,[]{return !loading;});
        if(current || retiring)return;
        loading=true;
    }
    auto session=std::make_shared<Session>();
    session->handle=dlopen("vendorGnssAdapter.so",RTLD_NOW|RTLD_LOCAL);
    if(session->handle) {
        auto* device=static_cast<GnssVendorDevice*>(dlsym(session->handle,"GnssVendorInterface"));
        if(device && device->size==sizeof(*device) && device->getGnssInterface) {
            auto* table=device->getGnssInterface();
            if(table && table->size==sizeof(*table) && table->enableGnss && table->disableGnss &&
               table->startGnss && table->stopGnss && table->injectsGnssReferenceInfo && table->setGnssConfigPara &&
               table->removeAuxiliaryData && table->injectExtendedEphemeris && table->getGnssModuleIface) {
                session->methods=*table;
            }
        }
        if(!session->methods.enableGnss){dlclose(session->handle);session->handle=nullptr;}
    }
    {
        std::lock_guard<std::mutex> lock(gate);
        if(session->handle)current=session;
        loading=false;changed.notify_all();
    }
}
const GnssVendorInterface* LocationVendorInterface::GetGnssVendorInterface() const
{
    std::lock_guard<std::mutex> lock(gate);
    if(!current)return nullptr;
    size_t index=(current->methods.getCachedLocationsSize?1:0)|(current->methods.flushCachedGnssLocations?2:0);
    return &proxies[index];
}
const void* LocationVendorInterface::GetModuleInterface(int id) const { return Module(id); }
void LocationVendorInterface::CleanUp()
{
    {
        std::unique_lock<std::mutex> lock(gate);
        changed.wait(lock,[]{return !loading;});
        if(current) {
            retiring=std::move(current);retiring->enabled=false;retiring->callbacks={};retireFailed=false;
            if(pthread_create(&reaper,nullptr,Reap,nullptr)==0)reaperStarted=true;
            else {retireFailed=true;changed.notify_all();}
        }
        if(callDepth || callbackDepth)return; // Reaper waits for this callback to return into/out of provider first.
        changed.wait(lock,[]{return !retiring || retireFailed;});
    }
    JoinCompleted();
}
} // namespace OHOS::HDI::Location
