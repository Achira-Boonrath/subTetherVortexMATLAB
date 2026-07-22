

#include <VxContent/KeyFrameManager.h>

#include <algorithm>
#include <iterator>

namespace Tutorials
{
    class KeyFrameListListener : public VxContent::KeyFrameList::Listener
    {
    public:
        KeyFrameListListener(ApplicationKeyFrameKeyboardListener* owner)
            : mOwner(owner)
        {
        }

        // Called when the restore key frame operation is completed.
        virtual void onRestored(VxContent::KeyFrameManager::eErrorCode code, const VxContent::KeyFrame& keyFrame, const std::string& msg)
        {
            if (code == VxContent::KeyFrameManager::kNoError)
            {
                mOwner->keyFrameRestored(keyFrame);
            }
        }

        // Pure virtual callback methods from class VxContent::KeyFrameList::Listener. Please refer to documentation for details.
        virtual void onSaved(VxContent::KeyFrameManager::eErrorCode code, const VxContent::KeyFrame& keyFrame) {}
        virtual void onRemoved(VxContent::KeyFrameManager::eErrorCode code, const VxContent::KeyFrame& keyFrame) {}
        virtual void onCleared() {}
        virtual void onStateChange(VxContent::KeyFrameList::eStatus) {}

    private:
        ApplicationKeyFrameKeyboardListener* mOwner;
    };

    // Constructor of a keyboard extension that responds to keyboard input to interact with the key frame functionality.
    ApplicationKeyFrameKeyboardListener::ApplicationKeyFrameKeyboardListener(VxSim::VxExtension* proxy)
        : VxSim::IExtension(proxy)
        , VxSim::IKeyboard(proxy)
        , mPreviouslyRestoredKeyFrameIndex(0)
        , mKeyFrameList()
    {
    }

    // Destructor
    ApplicationKeyFrameKeyboardListener::~ApplicationKeyFrameKeyboardListener()
    {
    }

    void ApplicationKeyFrameKeyboardListener::onActive()
    {
        mKeyFrameList = getApplicationContext().getKeyFrameManager()->createKeyFrameList("mechanism_viewer_tutorial", false /* key frames are not persisted */);
        if (mKeyFrameList->getStatus() != VxContent::KeyFrameList::kFeatureNotAvailable)
        {
            mListener = new KeyFrameListListener(this); // Creates a new key frame list but it is not initialized yet.
            mKeyFrameList->registerListener(mListener.get());

            addKeyDescription('o', "Save a new key frame");
            addKeyDescription('i', "Restore the last key frame");
            addKeyDescription('p', "Restore the previous key frame");
            addKeyDescription('l', "Restore the next key frame");
        }
    }

    void ApplicationKeyFrameKeyboardListener::onInactive()
    {
        mKeyFrameList->unregisterListener(mListener.get());
        mKeyFrameList = nullptr;
        mListener = nullptr;
    }

    // Key press invokes mapped key frame action.
    void ApplicationKeyFrameKeyboardListener::onKeyPressed(int key)
    {
        // A new key frame list is in an uninitialized state until the key frame module's internal data representation is initialized.
        // We check the state of our list before any operations on the list.
        if (mKeyFrameList->getStatus() == VxContent::KeyFrameList::kNotInitialized || 
            mKeyFrameList->getStatus() == VxContent::KeyFrameList::kFeatureNotAvailable)
        {
            return;
        }

        const Vx::VxArray<VxContent::KeyFrame>& keyFrames = mKeyFrameList->getKeyFrames();
        const size_t numberOfKeyFrames = keyFrames.size();

        switch (key)
        {
        case 'o': // Save a new key frame.
            mKeyFrameList->saveKeyFrame();
            break;
        case 'i': // Restore the last key frame saved.
            if (numberOfKeyFrames > 0)
            {
                mKeyFrameList->restore(keyFrames.back());
            }
            break;
        case 'p': // Restore the key frame that was saved before the last restored key frame.
            if (numberOfKeyFrames > 0)
            {
                if (mPreviouslyRestoredKeyFrameIndex >= 1)
                {
                    mKeyFrameList->restore(keyFrames[mPreviouslyRestoredKeyFrameIndex - 1]);
                }
                else
                {
                    // If there is no previous key frame, restore the first one.
                    mKeyFrameList->restore(keyFrames.front());
                }
            }
            break;
        case 'l': // Restore the key frame that was saved after the previously restored key frame.
            if (numberOfKeyFrames > 0)
            {
                if (mPreviouslyRestoredKeyFrameIndex < (keyFrames.size() - 1))
                {
                    mKeyFrameList->restore(keyFrames[mPreviouslyRestoredKeyFrameIndex + 1]);
                }
                else
                {
                    // If there is no next key frame, restore the last one.
                    mKeyFrameList->restore(keyFrames.back());
                }
            }
            break;
        default:
            break;
        }
    }

    // Called from the key frame list listener to notify the object that a key frame was restored.
    void ApplicationKeyFrameKeyboardListener::keyFrameRestored(const VxContent::KeyFrame& keyFrame)
    {
        const Vx::VxArray<VxContent::KeyFrame>& keyFrames = mKeyFrameList->getKeyFrames();
        
        // Find the restored key frame in the key frame list and update our index.
        for (int i = 0; i < keyFrames.size(); ++i)
        {
            if (keyFrames[i] == keyFrame)
            {
                mPreviouslyRestoredKeyFrameIndex = i;
                break;
            }
        }
    }
}
