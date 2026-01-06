import cv2
import os
import glob
import subprocess
import torch
from pathlib import Path
from basicsr.archs.rrdbnet_arch import RRDBNet
from realesrgan import RealESRGANer

'''
Package                 Version
----------------------- ---------------
absl-py                 2.3.1
addict                  2.4.0
basicsr                 1.4.2
certifi                 2026.1.4
charset-normalizer      3.4.4
colorama                0.4.6
contourpy               1.3.2
cv                      1.0.0
cycler                  0.12.1
facexlib                0.3.0
filelock                3.20.2
filterpy                1.4.5
fonttools               4.61.1
fsspec                  2025.12.0
future                  1.0.0
gfpgan                  1.3.8
grpcio                  1.76.0
idna                    3.11
ImageIO                 2.37.2
Jinja2                  3.1.6
kiwisolver              1.4.9
lazy_loader             0.4
llvmlite                0.46.0
lmdb                    1.7.5
Markdown                3.10
MarkupSafe              3.0.3
matplotlib              3.10.8
mpmath                  1.3.0
networkx                3.4.2
numba                   0.63.1
numpy                   1.24.0
opencv-contrib-python   4.12.0.88
opencv-python           4.12.0.88
packaging               25.0
pillow                  12.1.0
pip                     25.3
platformdirs            4.5.1
protobuf                6.33.2
pyparsing               3.3.1
python-dateutil         2.9.0.post0
PyYAML                  6.0.3
realesrgan              0.3.0
requests                2.32.5
scikit-image            0.25.2
scipy                   1.15.3
setuptools              65.5.0
six                     1.17.0
sympy                   1.14.0
tb-nightly              2.21.0a20251023
tensorboard-data-server 0.7.2
tifffile                2025.5.10
tomli                   2.3.0
torch                   1.13.1
torchvision             0.14.1
tqdm                    4.67.1
typing_extensions       4.15.0
urllib3                 2.6.2
Werkzeug                3.1.4
yapf                    0.43.0
'''
# --- CONFIGURATION ---
INPUT_DIR = "media"
OUTPUT_DIR = "media_upscaled"

# Real-ESRGAN Settings
MODEL_NAME = 'RealESRGAN_x4plus'  # Options: RealESRGAN_x4plus, RealESRNet_x4plus
SCALE = 4
TILE_SIZE = 0  # 0 for auto. Lower this (e.g., 400) if you run out of GPU memory (VRAM).

# Sharpening (Optional)
# Real-ESRGAN is naturally sharp. Only enable this if you want exaggerated edges.
APPLY_EXTRA_SHARPENING = False 
SHARPEN_AMOUNT = 0.5

def download_model_weights(model_name):
    """Downloads the pre-trained model weights if missing."""
    file_path = f'{model_name}.pth'
    if not os.path.exists(file_path):
        print(f"Model {model_name} not found. Downloading...")
        url = f'https://github.com/xinntao/Real-ESRGAN/releases/download/v0.1.0/{model_name}.pth'
        # Using wget via subprocess or system command is easiest, but let's use requests if available
        # Fallback to curl/wget for simplicity in script
        try:
            subprocess.run(['wget', url], check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
             try:
                 subprocess.run(['curl', '-L', url, '-o', file_path], check=True)
             except:
                 print(f"Error: Please download {url} manually and place it in this folder.")
                 exit()

def init_realesrgan():
    """Initialize the RealESRGANer instance."""
    
    # Check for GPU
    if not torch.cuda.is_available():
        print("WARNING: CUDA (GPU) not detected. Processing will be EXTREMELY slow.")
        device = torch.device('cpu')
    else:
        device = torch.device('cuda')

    download_model_weights(MODEL_NAME)

    # Define the model architecture (RRDBNet is standard for RealESRGAN)
    model = RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64, num_block=23, num_grow_ch=32, scale=4)
    
    upsampler = RealESRGANer(
        scale=SCALE,
        model_path=f'{MODEL_NAME}.pth',
        model=model,
        tile=TILE_SIZE,
        tile_pad=10,
        pre_pad=0,
        half=True if torch.cuda.is_available() else False, # Use FP16 if GPU available
        device=device
    )
    return upsampler

def sharpen_frame(image):
    """Applies a mild unsharp mask for extra crispness."""
    if not APPLY_EXTRA_SHARPENING:
        return image
    blurred = cv2.GaussianBlur(image, (0, 0), 1.0)
    sharpened = cv2.addWeighted(image, 1.0 + SHARPEN_AMOUNT, blurred, -SHARPEN_AMOUNT, 0)
    return sharpened

def process_image(path, output_folder, upsampler):
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, f"upscaled_{filename}")
    
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None: return

    try:
        # RealESRGANer returns (output, img_mode)
        output, _ = upsampler.enhance(img, outscale=SCALE)
        
        output = sharpen_frame(output)
        
        cv2.imwrite(save_path, output)
        print(f"[IMG] Finished: {filename}")
    except RuntimeError as e:
        print(f"[IMG] Failed {filename}: {e}. Try reducing TILE_SIZE.")

def process_video(path, output_folder, upsampler):
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, f"upscaled_{Path(filename).stem}.mp4")
    
    cap = cv2.VideoCapture(path)
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
        if not ret: break
        
        try:
            # enhance() expects RGB or BGR
            output, _ = upsampler.enhance(frame, outscale=SCALE)
            output = sharpen_frame(output)
            out.write(output)
        except RuntimeError as e:
            print(f"Error on frame {current_frame}: {e}. Try reducing TILE_SIZE.")
            break

        current_frame += 1
        print(f"      Progress: {current_frame}/{total_frames}", end='\r')
        
    cap.release()
    out.release()
    print(f"\n[VID] Finished: {filename}")
    
    merge_audio(path, save_path)

def merge_audio(original, new):
    """Merges audio using ffmpeg."""
    temp = new.replace(".mp4", "_audio.mp4")
    cmd = [
        'ffmpeg', '-y', '-loglevel', 'error',
        '-i', new, '-i', original,
        '-map', '0:v', '-map', '1:a',
        '-c:v', 'copy', '-c:a', 'copy', '-shortest', temp
    ]
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.exists(temp): os.replace(temp, new)
    except: pass

def main():
    if not os.path.exists(INPUT_DIR):
        print(f"Create directory '{INPUT_DIR}' and add files.")
        return
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    print("Initializing Real-ESRGAN (this may take a moment)...")
    upsampler = init_realesrgan()
    
    files = []
    for ext in ['*.jpg', '*.jpeg', '*.hevc', '*.h265', '*.mp4']:
        files.extend(glob.glob(os.path.join(INPUT_DIR, ext)))
        
    print(f"Found {len(files)} files.")
    
    for file_path in files:
        if file_path.lower().endswith(('.jpg', '.jpeg')):
            process_image(file_path, OUTPUT_DIR, upsampler)
        elif file_path.lower().endswith(('.hevc', '.h265', '.mp4')):
            process_video(file_path, OUTPUT_DIR, upsampler)

if __name__ == "__main__":
    main()