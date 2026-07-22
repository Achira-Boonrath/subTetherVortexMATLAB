// Modified by Andreas Enzenhöfer, September 21, 2016: extensions for handling graphics

#include "MyLibraryNet/AccessoryModule.h"


AccessoryModule::~AccessoryModule()
    {
        // does nothing
    }

AccessoryModule::AccessoryModule(VxSim::VxSimulatorModule * proxy)
        : VxSim::ISimulatorModule(proxy)
    {
        // does nothing
    }

// Implements code when the this user extension is added to the Frameworks application.
// It adds the extension to the list of owned extensions if the type is MyExtension.
void AccessoryModule::onExtensionAdded(VxSim::VxExtension * extension)
    {
        VxSim::VxSmartInterface<VxGraphics::IAccessory> accessory = extension;
        if(accessory.valid())
        {
            mAccessories.insert(accessory);
        }
    }

// Implements code when this user extension is removed from the Frameworks application.
// It removes the extension from the list of owned extensions if the type is MyExtension.
void AccessoryModule::onExtensionRemoved(VxSim::VxExtension * extension)
    {
        VxSim::VxSmartInterface<VxGraphics::IAccessory> accessory = extension;
        if(accessory.valid())
        {
            mAccessories.clear();
            //remove(accessory);
        }
    }


// Called after modules Update().
// Call all MyExtension updatePosition if enabled
void AccessoryModule::onPostUpdate(VxSim::eApplicationMode mode)
    {
        for(auto i = mAccessories.begin(); i != mAccessories.end(); ++i)
        {
            (*i)->setAccessoryVisible(true);
        }
    }