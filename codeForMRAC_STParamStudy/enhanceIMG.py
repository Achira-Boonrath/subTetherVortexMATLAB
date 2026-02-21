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
import cv2
import os
import glob
import subprocess
import torch
from pathlib import Path
from basicsr.archs.rrdbnet_arch import RRDBNet
from realesrgan import RealESRGANer

# --- CONFIGURATION ---
# INPUT_DIR: place source images/videos here
INPUT_DIR = "media"
# OUTPUT_DIR: enhanced/upscaled outputs will be written here
OUTPUT_DIR = "media_upscaled"

# Real-ESRGAN Settings
# MODEL_NAME: filename (without path) for the pretrained weights.
# The script will attempt to download MODEL_NAME.pth if it's not present.
MODEL_NAME = 'RealESRGAN_x4plus'
# SCALE: integer upscale factor applied to width/height (e.g. 4 => 4x upscaling)
SCALE = 4
# TILE_SIZE: process image in tiles if you run out of GPU memory.
# 0 disables tiling (preferred when enough VRAM). Use 400/200 to reduce memory footprint.
TILE_SIZE = 0

# Optional sharpening applied after upscaling
APPLY_EXTRA_SHARPENING = False
# SHARPEN_AMOUNT: positive float. Larger values produce stronger sharpening.
SHARPEN_AMOUNT = 0.5

def download_model_weights(model_name):
    """
    Ensure the required .pth model weights file exists locally.
    If not, attempt to download it using 'wget' or 'curl'.
    If both utilities are unavailable, prompt the user to download manually.
    """
    file_path = f'{model_name}.pth'
    if not os.path.exists(file_path):
        print(f"Model {model_name} not found. Downloading...")
        url = f'https://github.com/xinntao/Real-ESRGAN/releases/download/v0.1.0/{model_name}.pth'
        try:
            # Try wget first (common on Linux)
            subprocess.run(['wget', url], check=True)
        except:
             try:
                 # Fallback to curl (macOS and some Linux systems)
                 subprocess.run(['curl', '-L', url, '-o', file_path], check=True)
             except:
                 # If both fail, instruct user to download manually — avoid crashing with cryptic error
                 print(f"Error: Please download {url} manually.")
                 exit()

def init_realesrgan():
    """
    Initialize Real-ESRGAN model and RealESRGANer wrapper.
    - Selects device (CUDA if available, otherwise CPU).
    - Downloads model weights if missing.
    - Constructs RRDBNet architecture with parameters matching the selected Real-ESRGAN model.
    - Wraps model in RealESRGANer which handles preprocessing, tiling, and inference.
    Returns the configured upsampler instance.
    """
    # Choose device: prefer GPU for speed if CUDA is available
    if not torch.cuda.is_available():
        print("WARNING: CUDA (GPU) not detected. Processing will be slow.")
        device = torch.device('cpu')
    else:
        device = torch.device('cuda')

    # Ensure weights exist before constructing the model wrapper
    download_model_weights(MODEL_NAME)

    # RRDBNet architecture parameters chosen to match shipped Real-ESRGAN model
    # Adjusting these without matching weights will break inference.
    model = RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64, num_block=23, num_grow_ch=32, scale=4)
    
    # RealESRGANer handles:
    # - Loading weights into the model
    # - Optional tiling to reduce memory usage
    # - Converting between color spaces and handling alpha channels transparently
    upsampler = RealESRGANer(
        scale=SCALE,
        model_path=f'{MODEL_NAME}.pth',
        model=model,
        tile=TILE_SIZE,
        tile_pad=10,     # overlap padding between tiles to avoid seams
        pre_pad=0,       # extra padding to avoid border artifacts
        half=True if torch.cuda.is_available() else False,  # use FP16 on CUDA to save memory / speed
        device=device
    )
    return upsampler

def sharpen_frame(image):
    """
    Optionally apply light sharpening to the upscaled image.
    Uses unsharp masking: mix original and a Gaussian-blurred copy.
    - image: NumPy array in BGR(A) format (as returned by OpenCV)
    Returns the possibly sharpened image (same dtype as input).
    """
    if not APPLY_EXTRA_SHARPENING:
        return image
    # Gaussian blur with sigma=1.0; kernel size 0 lets OpenCV compute it from sigma
    blurred = cv2.GaussianBlur(image, (0, 0), 1.0)
    # addWeighted: result = src1*alpha + src2*beta + gamma
    # Here we add (1+amount)*original + (-amount)*blurred to emphasize edges
    sharpened = cv2.addWeighted(image, 1.0 + SHARPEN_AMOUNT, blurred, -SHARPEN_AMOUNT, 0)
    return sharpened

def process_image(path, output_folder, upsampler):
    """
    Read an image, run Real-ESRGAN upscaling, optional sharpening, and save result.
    Handles images with alpha (transparency) via cv2.IMREAD_UNCHANGED.
    - path: input file path
    - output_folder: directory to save processed image
    - upsampler: RealESRGANer instance
    """
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, f"upscaled_{filename}")
    
    # Read with IMREAD_UNCHANGED to retain alpha channel for PNGs if present.
    # This ensures RGBA images are preserved and RealESRGANer can handle them.
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        # Could not read file (corrupt / unsupported). Skip silently.
        return

    try:
        # RealESRGANer.enhance returns (output_image, has_aligned)
        # outscale=SCALE ensures the final image is scaled by the specified factor.
        output, _ = upsampler.enhance(img, outscale=SCALE)
        
        # Optionally sharpen the result to enhance perceived detail
        output = sharpen_frame(output)
        
        # Write result to disk. OpenCV will pick file format by extension.
        cv2.imwrite(save_path, output)
        print(f"[IMG] Finished: {filename}")
    except RuntimeError as e:
        # Commonly raised when GPU runs out of memory or tiling parameters are incompatible.
        # Suggest lowering TILE_SIZE when this happens.
        print(f"[IMG] Failed {filename}: {e}. Try reducing TILE_SIZE.")

def process_video(path, output_folder, upsampler):
    """
    Read video frame-by-frame, upscale each frame, write to an output mp4,
    then attempt to copy audio from the original into the upscaled video using ffmpeg.
    Notes:
    - This implementation keeps everything simple and streams frames through RealESRGANer.
    - For large videos, processing will be slow and GPU memory constrained. Use tiling in that case.
    """
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, f"upscaled_{Path(filename).stem}.mp4")
    
    cap = cv2.VideoCapture(path)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Compute new resolution based on SCALE
    new_width = width * SCALE
    new_height = height * SCALE
    
    # Create a VideoWriter for the upscaled frames.
    # Using mp4v codec: widely compatible, but change if you need other containers/codecs.
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(save_path, fourcc, fps, (new_width, new_height))
    
    print(f"[VID] Processing {filename} ({total_frames} frames)...")
    
    current_frame = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            # End of stream or read error
            break
        
        try:
            # Upscale this single frame. RealESRGANer returns the upscaled frame (BGR).
            output, _ = upsampler.enhance(frame, outscale=SCALE)
            output = sharpen_frame(output)
            out.write(output)
        except RuntimeError as e:
            # When a frame fails (often OOM), report and stop further processing.
            print(f"Error on frame {current_frame}: {e}")
            break

        current_frame += 1
        # Overwrite the same console line with progress for visual feedback
        print(f"      Progress: {current_frame}/{total_frames}", end='\r')
        
    cap.release()
    out.release()
    print(f"\n[VID] Finished: {filename}")
    # Try to restore original audio track into the upscaled video
    merge_audio(path, save_path)

def merge_audio(original, new):
    """
    Use ffmpeg to copy audio stream from the original video into the new upscaled video.
    - original: path to source file that may contain audio
    - new: path to upscaled video (video-only)
    The function creates a temporary file with audio merged, then replaces the upscaled file.
    """
    temp = new.replace(".mp4", "_audio.mp4")
    # ffmpeg mapping:
    # -map 0:v : take video from first input (the upscaled file)
    # -map 1:a : take audio from second input (original file)
    # -c:v copy, -c:a copy : do not re-encode, just remux streams
    cmd = [
        'ffmpeg', '-y', '-loglevel', 'error',
        '-i', new, '-i', original,
        '-map', '0:v', '-map', '1:a',
        '-c:v', 'copy', '-c:a', 'copy', '-shortest', temp
    ]
    try:
        # Suppress ffmpeg output; if it fails, we proceed without audio
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.exists(temp):
            # Replace the upscaled file with the one that includes audio
            os.replace(temp, new)
    except:
        # Any ffmpeg errors are non-fatal for the upscaling task
        pass

def main():
    """
    Main entry point:
    - Ensure input directory exists and output directory is created.
    - Initialize Real-ESRGAN model once.
    - Scan the input directory for supported file extensions.
    - Dispatch files to image or video processing routines.
    """
    if not os.path.exists(INPUT_DIR):
        print(f"Create directory '{INPUT_DIR}' and add files.")
        return
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    print("Initializing Real-ESRGAN...")
    upsampler = init_realesrgan()
    
    # Extensions to search for. Add/remove types as needed.
    extensions = ['*.jpg', '*.jpeg', '*.heic', '*.png', '*.hevc', '*.h265', '*.mp4']
    files = []
    for ext in extensions:
        # glob returns matching filepaths in INPUT_DIR for each extension
        files.extend(glob.glob(os.path.join(INPUT_DIR, ext)))
        
    print(f"Found {len(files)} files.")
    
    for file_path in files:
        # Route images and videos to their respective handlers.
        # We include '.png' explicitly to process transparency-enabled images.
        if file_path.lower().endswith(('.jpg', '.jpeg', '.png', '.heic')):
            process_image(file_path, OUTPUT_DIR, upsampler)
        elif file_path.lower().endswith(('.hevc', '.h265', '.mp4')):
            process_video(file_path, OUTPUT_DIR, upsampler)

if __name__ == "__main__":
    main()