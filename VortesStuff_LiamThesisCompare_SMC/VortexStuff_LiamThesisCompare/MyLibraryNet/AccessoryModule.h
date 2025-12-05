#ifndef _ACCESSORY_MODULE_H_
#define _ACCESSORY_MODULE_H_

#include <VxSim/ISimulatorModule.h>
#include <VxSim/VxSmartInterface.h>
#include <VxGraphics/IAccessory.h>
#include <VxSim/VxExtensionFactory.h>
#include <set>

class AccessoryModule : public VxSim::ISimulatorModule
{

    std::set<VxSim::VxSmartInterface<VxGraphics::IAccessory>> mAccessories;

public:

    ~AccessoryModule();

    AccessoryModule(VxSim::VxSimulatorModule * proxy);

    virtual void onExtensionAdded(VxSim::VxExtension * extension);

    virtual void onExtensionRemoved(VxSim::VxExtension * extension);

    virtual void onPostUpdate(VxSim::eApplicationMode mode);
};

#endif //_ACCESSORY_MODULE_H_