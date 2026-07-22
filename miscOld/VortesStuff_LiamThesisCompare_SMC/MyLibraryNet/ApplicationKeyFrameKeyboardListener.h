#ifndef __APPLICATION_KEY_FRAME_KEYBOARD_LISTENER_HEADER__
#define __APPLICATION_KEY_FRAME_KEYBOARD_LISTENER_HEADER__

#include <VxSim/IKeyboard.h>
#include <VxSim/IExtension.h>
#include <VxSim/VxFactoryKey.h>
#include <Vx/VxSmartPtr.h>
#include <VxContent/KeyFrameList.h>
#include <VxContent/KeyFrame.h>

const VxSim::VxFactoryKey kApplicationKeyFrameKeyboardListenerKey(VxSim::VxUuid("{ED5D05C4-F118-4ACA-AA6D-CA1A204FFC30}"), "Tutorials", "ApplicationKeyFrameKeyboardListener");

using namespace Vx;
using namespace VxContent;
using namespace VxDynamics;
using namespace VxSim;
using namespace CableSystems::DynamicsICD;

{
    class KeyFrameListListener;

    // This class listens for specific keyboard key events that triggers saving and restoring key frames.
    // A key frame is a capture of all application content data that allows the application to resume the simulation from a specific point in time.
    // Key frames are aggregated in a key frame list.
    // This simple implementation allow you to save a key frame to memory, reload the latest key frame and travel down the list of previously saved key frames.
    class ApplicationKeyFrameKeyboardListener : public VxSim::IKeyboard, public VxSim::IExtension
    {
    public:
        ApplicationKeyFrameKeyboardListener(VxSim::VxExtension* proxy);
        virtual ~ApplicationKeyFrameKeyboardListener();

        // Method overloaded from VxSim::IExtension
        void onActive();
        void onInactive();

        // Method overloaded from VxSim::IKeyboard
        virtual void onKeyPressed(int key);

        void keyFrameRestored(const VxContent::KeyFrame& keyFrame);

    private:
        Vx::VxSmartPtr<VxContent::KeyFrameList> mKeyFrameList;
        size_t mPreviouslyRestoredKeyFrameIndex;
        Vx::VxSmartPtr<KeyFrameListListener> mListener;
    };
}

#endif //__APPLICATION_KEY_FRAME_KEYBOARD_LISTENER_HEADER__
