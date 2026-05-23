import torch

print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"CUDA version: {torch.version.cuda}")

if not torch.cuda.is_available():
    try:
        # This often gives a more detailed error message
        torch.cuda.init()
    except Exception as e:
        print(f"Initialization Error: {e}")
