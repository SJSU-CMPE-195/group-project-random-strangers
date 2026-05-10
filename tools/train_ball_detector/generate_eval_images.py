from pathlib import Path
import argparse
import random


ROOT = Path("datasets")
SCRIPT_DIR = Path(__file__).resolve().parent
DATASET_ROOT = ROOT / "coco_person_ball_subset"
OUT = DATASET_ROOT / "yolo"
EVAL_LIST = DATASET_ROOT / "eval_images.txt"


def main():
    parser = argparse.ArgumentParser(description="Generate eval_images.txt from a split folder.")
    parser.add_argument("--split", default="val2017", help="Dataset split to sample from.")
    parser.add_argument("--count", type=int, default=50, help="Number of images to include.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for sampling.")
    parser.add_argument("--run-dir", default="", help="Run directory to write eval_images.txt into.")
    parser.add_argument("--out", default="", help="Output file path (overrides --run-dir).")
    args = parser.parse_args()

    split_dir = OUT / "images" / args.split
    if not split_dir.exists():
        parser.error(f"Missing split directory: {split_dir}")

    images = sorted(p for p in split_dir.iterdir() if p.suffix.lower() == ".jpg")
    if not images:
        parser.error(f"No .jpg files found in: {split_dir}")

    count = max(0, args.count)
    if count and count < len(images):
        rng = random.Random(args.seed)
        images = rng.sample(images, count)

    if args.out:
        out_path = Path(args.out)
    elif args.run_dir:
        out_path = Path(args.run_dir) / "eval_images.txt"
    else:
        out_path = SCRIPT_DIR / "eval_images.txt"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rel_paths = [p.relative_to(OUT).as_posix() for p in images]
    out_path.write_text("\n".join(rel_paths) + "\n")

    print(f"Wrote {len(rel_paths)} entries to {out_path}")


if __name__ == "__main__":
    main()
