# GraspNet-Lite

Lightweight sandbox for experimenting with coupled object classification and robotic grasp-type prediction on a small image collection (`apple`, `bottle`, `egg`, `grape`, `mug`). Each object class maps deterministically to a grasp label (`spherical`, `cylindrical`, `tripod`, `pinch`, `hook`), so every training script exposes two heads: object and grip.

## Repository Layout
- `data/raw/<object>/*.jpg` - source JPEG/PNG images grouped by object class.
- `data/splits.json` - train/val/test manifest created by `src/create_splits.py`.
- `src/create_splits.py` - scans `data/raw`, keeps per-class balance, and writes the manifest.
- `src/train_baselines.py` - five classical trials (logistic regression and SVM) on flattened RGB features.
- `src/train_mlp.py` - five two-head MLP trials with configurable hidden sizes and activations.
- `src/train_cnn.py` - PyTorch two-head CNN with a shared backbone plus two classifiers.
- `plots_*` - automatically generated plots (confusion matrices, precision/recall bars, curves).
- `environment.yml` - conda specification for Python, PyTorch, scikit-learn, and plotting packages.

## Environment Setup
```bash
conda env create -f environment.yml
conda activate graspnet_lite
```
The environment ships CPU dependencies plus CUDA 11.8 builds of PyTorch and TorchVision. If you only have CPU, remove `pytorch-cuda` from the YAML or install PyTorch separately following the official instructions. All scripts run on CPU, but `src/train_cnn.py` automatically uses CUDA when present.

## Dataset Expectations
Organize images as:
```
data/
  raw/
    apple/
      *.jpg|jpeg|png
    bottle/
    egg/
    grape/
    mug/
```
Add or replace images freely; the scripts do not assume fixed filenames. The `OBJECT_TO_GRIP` mapping (object to grasp label) is defined near the top of every training script. Edit it there if you introduce new classes.

## Creating Splits
Run `src/create_splits.py` to build a balanced manifest:
```bash
python src/create_splits.py \
  --raw-dir data/raw \
  --output data/splits.json \
  --train-ratio 0.7 \
  --val-ratio 0.15 \
  --seed 42
```
`data/splits.json` stores entries such as:
```json
{
  "path": "data/raw/apple/img_p1_88.jpeg",
  "label": "apple"
}
```
Downstream scripts only read this manifest, so you can regenerate it to adjust split ratios, seeds, or even change the raw directory layout without modifying the training code.

## Running the Baseline Models
The baseline script runs five hard-coded trials (two logistic regression settings and three SVMs) and logs both object and grip performance:
```bash
python src/train_baselines.py \
  --splits data/splits.json \
  --image-size 128 128 \
  --plots-dir plots_baselines
```
Key details:
- Images are resized, flattened, and normalized to `[0, 1]`.
- Separate LogisticRegression/SVC models are trained for object and grip outputs.
- Each trial writes confusion matrices and precision/recall bar plots for val/test sets, plus a summary bar chart of test accuracy.

## Running the Two-Head MLPs
The MLP script mirrors the baselines but swaps in `sklearn.neural_network.MLPClassifier` and sweeps five hyperparameter sets:
```bash
python src/train_mlp.py \
  --splits data/splits.json \
  --image-size 128 128 \
  --plots-dir plots_mlp
```
It builds the same flattened features, launches a dedicated MLP for each head, and saves per-trial visualizations together with `plots_mlp/mlp_trials_test_accuracy.png`.

## Running the CNN Experiments
`src/train_cnn.py` introduces a shared convolutional backbone with two classification heads:
```bash
python src/train_cnn.py \
  --splits data/splits.json \
  --image-size 128 \
  --epochs 20 \
  --batch-size 32 \
  --plots-dir plots_cnn
```
Highlights:
- Uses torchvision transforms (resize, tensor conversion, ImageNet normalization).
- Five predefined trials vary convolutional filter stacks, kernel sizes, activations (ReLU and LeakyReLU), and learning rates.
- During training it prints epoch-wise metrics and records loss/error curves.
- After each trial it emits classification reports, confusion matrices, and precision/recall bars for both object and grip heads on val/test sets.
- Automatically selects GPU when available.

## Outputs and Logging
- `plots_baselines/trial_*`, `plots_mlp/trial_*`, `plots_cnn/trial_*` - confusion matrices, precision/recall bars, and (for CNN) loss/error curves.
- `plots_baselines/baseline_trials_test_accuracy.png` and `plots_mlp/mlp_trials_test_accuracy.png` - side-by-side test accuracy comparisons across trials.
- Console output contains scikit-learn `classification_report` tables plus summary lines per trial. Use shell redirection to save logs if you need persistent text artifacts.

## Tips
- Regenerate `data/splits.json` whenever the raw dataset changes so class balance remains intact.
- Override CLI defaults (`--image-size`, `--epochs`, ratios, etc.) to run ablations; every script prints its parsed configuration at launch.
- When running on Windows without system-wide CUDA, install the CUDA Toolkit that matches `pytorch-cuda=11.8` or drop that dependency if you only plan to run on CPU.
