import cv2
from cv2 import dnn_superres
import os
import glob
import subprocess
from pathlib import Path

# --- CONFIGURATION ---
INPUT_DIR = "media"
OUTPUT_DIR = "media_upscaled"
MODEL_PATH = "EDSR_x4.pb"   # Path to your .pb model file
MODEL_NAME = "edsr"         # specific algorithm name: 'edsr', 'fsrcnn', 'lapsrn', 'espcn'
SCALE = 4                   # Upscale factor: 2, 3, 4, or 8 (must match the model file)
USE_CUDA = False            # Set to True if you have an NVIDIA GPU and CUDA-enabled OpenCV

def init_model():
    """Initializes the super resolution model."""
    print(f"Loading model: {MODEL_NAME} x{SCALE}...")
    try:
        sr = dnn_superres.DnnSuperResImpl_create()
        sr.readModel(MODEL_PATH)
        sr.setModel(MODEL_NAME, SCALE)
        if USE_CUDA:
            sr.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            sr.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        return sr
    except Exception as e:
        print(f"Failed to load model. Ensure {MODEL_PATH} exists and is correct.")
        raise e

def upscale_image(path, output_folder, sr):
    """Upscales a JPG image."""
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, f"upscaled_{filename}")
    
    img = cv2.imread(path)
    if img is None:
        print(f"Skipping corrupt image: {filename}")
        return

    result = sr.upsample(img)
    cv2.imwrite(save_path, result)
    print(f"[IMG] Processed: {filename}")

def upscale_video(path, output_folder, sr):
    """Upscales a HEVC video."""
    filename = os.path.basename(path)
    # Change extension to .mp4 for better compatibility after processing
    save_path = os.path.join(output_folder, f"upscaled_{Path(filename).stem}.mp4")
    
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"Skipping unreadable video: {filename}")
        return

    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Calculate new dimensions
    new_width = width * SCALE
    new_height = height * SCALE
    
    # Setup Video Writer (using mp4v codec)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(save_path, fourcc, fps, (new_width, new_height))
    
    print(f"[VID] Processing {filename} ({total_frames} frames)...")
    
    current_frame = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        upscaled_frame = sr.upsample(frame)
        out.write(upscaled_frame)
        
        current_frame += 1
        print(f"      Progress: {current_frame}/{total_frames}", end='\r')

    cap.release()
    out.release()
    print(f"\n[VID] Finished: {filename}")
    
    # Attempt to merge original audio back using ffmpeg (if installed)
    merge_audio(path, save_path)

def merge_audio(original_video, new_video):
    """Merges audio from original video to the new silent upscaled video."""
    temp_file = new_video.replace(".mp4", "_audio.mp4")
    
    # Simple ffmpeg command: copy video from new, copy audio from original
    cmd = [
        'ffmpeg', '-y',
        '-i', new_video,
        '-i', original_video,
        '-map', '0:v', '-map', '1:a',
        '-c:v', 'copy', '-c:a', 'copy',
        '-shortest',
        temp_file
    ]
    
    try:
        # Check if ffmpeg is in path by running it with subprocess
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.exists(temp_file):
            os.replace(temp_file, new_video)
            print("      Audio merged successfully.")
    except FileNotFoundError:
        print("      Note: FFmpeg not found. Video saved without audio.")

def main():
    # Setup directories
    if not os.path.exists(INPUT_DIR):
        print(f"Error: Directory '{INPUT_DIR}' not found.")
        return
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Initialize Super Resolution
    sr = init_model()
    
    # Find files
    extensions = ['*.jpg', '*.jpeg', '*.hevc', '*.h265']
    files = []
    for ext in extensions:
        files.extend(glob.glob(os.path.join(INPUT_DIR, ext)))
        
    print(f"Found {len(files)} files in '{INPUT_DIR}'")
    
    for file_path in files:
        if file_path.lower().endswith(('.jpg', '.jpeg')):
            upscale_image(file_path, OUTPUT_DIR, sr)
        elif file_path.lower().endswith(('.hevc', '.h265')):
            upscale_video(file_path, OUTPUT_DIR, sr)
            
    print("\nBatch processing complete!")

if __name__ == "__main__":
    main()