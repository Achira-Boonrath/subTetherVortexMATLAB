import cv2
from cv2 import dnn_superres
import os
import glob
import subprocess
from pathlib import Path

#pip install opencv-contrib-python#

# --- CONFIGURATION ---
INPUT_DIR = "media"
OUTPUT_DIR = "media_upscaled"

# Model Settings
MODEL_PATH = "EDSR_x4.pb"   # Path to your .pb model file
MODEL_NAME = "edsr"         # 'edsr', 'fsrcnn', 'lapsrn', 'espcn'
SCALE = 4                   # Must match the model file (x2, x3, x4)
USE_CUDA = False            # Set True if you have an NVIDIA GPU

# Sharpening Settings
APPLY_SHARPENING = True
SHARPEN_AMOUNT = 1.2       # Strength: 1.0 is standard, higher is stronger
SHARPEN_SIGMA = 1.0        # Blur radius for the mask (1.0 is usually good)

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
        print(f"Failed to load model. Ensure {MODEL_PATH} exists.")
        raise e

def sharpen_frame(image):
    """
    Applies Unsharp Masking to sharpen the image.
    Formula: sharpened = original + (original - blurred) * amount
    """
    if not APPLY_SHARPENING:
        return image
        
    # Create a Gaussian blur version of the image
    blurred = cv2.GaussianBlur(image, (0, 0), SHARPEN_SIGMA)
    
    # Calculate the weighted sum
    # Formula: src1 * alpha + src2 * beta + gamma
    # We want: Original * (1 + Amount) + Blurred * (-Amount)
    sharpened = cv2.addWeighted(image, 1.0 + SHARPEN_AMOUNT, blurred, -SHARPEN_AMOUNT, 0)
    
    return sharpened

def upscale_image(path, output_folder, sr):
    """Upscales and sharpens a JPG image."""
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, f"upscaled_{filename}")
    
    img = cv2.imread(path)
    if img is None:
        print(f"Skipping corrupt image: {filename}")
        return

    # 1. Upscale
    upscaled = sr.upsample(img)
    
    # 2. Sharpen
    final_img = sharpen_frame(upscaled)
    
    cv2.imwrite(save_path, final_img)
    print(f"[IMG] Processed & Sharpened: {filename}")

def upscale_video(path, output_folder, sr):
    """Upscales and sharpens a HEVC video."""
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, f"upscaled_{Path(filename).stem}.mp4")
    
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"Skipping unreadable video: {filename}")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    new_width = width * SCALE
    new_height = height * SCALE
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(save_path, fourcc, fps, (new_width, new_height))
    
    print(f"[VID] Processing {filename} ({total_frames} frames)...")
    
    current_frame = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 1. Upscale
        upscaled_frame = sr.upsample(frame)
        
        # 2. Sharpen
        final_frame = sharpen_frame(upscaled_frame)
        
        out.write(final_frame)
        
        current_frame += 1
        print(f"      Progress: {current_frame}/{total_frames}", end='\r')

    cap.release()
    out.release()
    print(f"\n[VID] Finished: {filename}")
    
    merge_audio(path, save_path)

def merge_audio(original_video, new_video):
    """Merges audio from original video to the new silent video using ffmpeg."""
    temp_file = new_video.replace(".mp4", "_audio.mp4")
    
    # Check if original has audio stream first (optional optimization)
    cmd = [
        'ffmpeg', '-y', '-loglevel', 'error',
        '-i', new_video,
        '-i', original_video,
        '-map', '0:v', '-map', '1:a',
        '-c:v', 'copy', '-c:a', 'copy',
        '-shortest',
        temp_file
    ]
    
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.exists(temp_file):
            os.replace(temp_file, new_video)
            print("      Audio merged successfully.")
    except FileNotFoundError:
        print("      Note: FFmpeg not found. Video saved without audio.")

def main():
    if not os.path.exists(INPUT_DIR):
        print(f"Error: Directory '{INPUT_DIR}' not found.")
        return
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    sr = init_model()
    
    # Extensions to look for
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