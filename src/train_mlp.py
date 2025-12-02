"""
Trains and evaluates a Multi-Layer Perceptron (MLP) classifier for image classification.

This script uses a pre-defined data split from a JSON file to train an MLP model,
evaluates its performance on validation and test sets, and saves detailed classification
metrics and plots. It is designed to be configurable via command-line arguments.
"""

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from PIL import Image
from sklearn.metrics import (
    accuracy_score,
    classification_report,
    confusion_matrix,
    precision_recall_fscore_support,
)
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import LabelEncoder

# Fixed mapping object -> grip
OBJECT_TO_GRIP = {
    "apple": "spherical",
    "bottle": "cylindrical",
    "egg": "tripod",
    "grape": "pinch",
    "mug": "hook",
}


def load_splits(path: Path) -> Dict[str, List[Dict[str, str]]]:
    """Loads data splits from a JSON file."""
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_image(path: Path, size: Tuple[int, int]) -> np.ndarray:
    """Loads, converts, resizes, and normalizes an image."""
    img = Image.open(path).convert("RGB")
    img = img.resize(size)
    arr = np.asarray(img, dtype=np.float32) / 255.0
    return arr.reshape(-1)


def build_dataset(
    entries: List[Dict[str, str]],
    image_size: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray]:
    """Builds a dataset from a list of entries."""
    xs: List[np.ndarray] = []
    obj_labels: List[str] = []
    grip_labels: List[str] = []
    for item in entries:
        img_path = Path(item["path"])
        obj_label = item["label"]
        grip_label = OBJECT_TO_GRIP[obj_label]
        xs.append(load_image(img_path, image_size))
        obj_labels.append(obj_label)
        grip_labels.append(grip_label)
    x_arr = np.stack(xs, axis=0)
    return x_arr, np.array(obj_labels), np.array(grip_labels)


def plot_confusion(
    title_prefix: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str = "",
) -> None:
    """Plots a normalized confusion matrix."""
    cm = confusion_matrix(y_true, y_pred, labels=np.arange(len(class_names)))
    cm_norm = cm.astype(np.float32) / cm.sum(axis=1, keepdims=True)

    fig, ax = plt.subplots(figsize=(8, 6))
    sns.heatmap(
        cm_norm,
        annot=True,
        fmt=".2f",
        cmap="Blues",
        xticklabels=class_names,
        yticklabels=class_names,
        cbar_kws={"label": "Normalized frequency"},
        ax=ax,
    )
    ax.set_xlabel("Predicted label")
    ax.set_ylabel("True label")
    extra = f"\n{title_suffix}" if title_suffix else ""
    ax.set_title(f"{title_prefix} - {split_name} confusion matrix{extra}")
    plt.tight_layout()
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{model_name.replace(' ', '_')}_{split_name}_confusion.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def plot_precision_recall(
    title_prefix: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    class_names: List[str],
    output_dir: Path,
    split_name: str,
    title_suffix: str = "",
) -> None:
    """Plots per-class precision and recall as grouped bars."""
    indices = np.arange(len(class_names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(indices - width / 2, precision, width, label="Precision")
    ax.bar(indices + width / 2, recall, width, label="Recall")
    ax.set_xticks(indices)
    ax.set_xticklabels(class_names, rotation=30, ha="right")
    ax.set_ylabel("Score")
    ax.set_ylim(0.0, 1.05)
    extra = f"\n{title_suffix}" if title_suffix else ""
    ax.set_title(f"{title_prefix} - {split_name.capitalize()} precision/recall{extra}")
    ax.legend(loc="lower right")
    plt.tight_layout()
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{model_name.replace(' ', '_')}_{split_name}_precision_recall.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def train_and_report(
    model,
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    x_test: np.ndarray,
    y_test: np.ndarray,
    class_names: List[str],
    title_prefix: str,
    plots_dir: Path,
    params: Dict[str, Any],
) -> None:
    """Trains a model and evaluates it on validation and test sets."""
    print(f"=== Training {name} ===")
    model.fit(x_train, y_train)

    def eval_split(split_name: str, x: np.ndarray, y_true: np.ndarray) -> Dict[str, Any]:
        y_pred = model.predict(x)
        acc = accuracy_score(y_true, y_pred)
        report = classification_report(
            y_true, y_pred, target_names=class_names, digits=4
        )
        precision, recall, _, _ = precision_recall_fscore_support(
            y_true, y_pred, labels=np.arange(len(target_names))
        )
        return {"y_true": y_true, "y_pred": y_pred, "precision": precision, "recall": recall}

    title_suffix = ", ".join(f"{k}={v}" for k, v in params.items())

    results["val"] = report_split("val", x_val, y_val)
    results["test"] = report_split("test", x_test, y_test)

    # Generate and save plots
    for split_name in ("val", "test"):
        split_res = results[split_name]
        plot_confusion(
            title_prefix,
            y_true,
            y_pred,
            class_names,
            plots_dir,
            split_name,
            title_suffix=title_suffix,
        )
        plot_precision_recall(
            title_prefix,
            y_true,
            y_pred,
            class_names,
            plots_dir,
            split_name,
            title_suffix=title_suffix,
        )
        return {"accuracy": acc, "report": report}


def main() -> None:
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description="Train an MLP classifier on an image dataset."
    )
    parser.add_argument(
        "--splits", type=str, default="data/splits.json", help="Path to data splits JSON file."
    )
    parser.add_argument(
        "--image-size", type=int, nargs=2, default=(128, 128), metavar=("W", "H"), help="Image resize dimensions."
    )
    parser.add_argument(
        "--plots-dir", type=str, default="plots", help="Directory to save evaluation plots."
    )
    parser.add_argument(
        "--hidden-layer-sizes", type=int, nargs="+", default=[100, 50], help="Sizes of hidden layers."
    )
    parser.add_argument(
        "--activation", type=str, default="relu", help="Activation function for hidden layers."
    )
    parser.add_argument(
        "--solver", type=str, default="adam", help="The solver for weight optimization."
    )
    parser.add_argument(
        "--learning-rate-init", type=float, default=0.001, help="Initial learning rate."
    )
    parser.add_argument(
        "--max-iter", type=int, default=1000, help="Maximum number of iterations."
    )
    args = parser.parse_args()

    # Validate paths
    splits_path = Path(args.splits)
    if not splits_path.exists():
        raise SystemExit(f"Splits file not found: {splits_path}")

    # Load data
    splits = load_splits(splits_path)
    image_size = tuple(args.image_size)
    plots_dir = Path(args.plots_dir)

    print("Building datasets...")
    x_train, y_train_str = build_dataset(splits["train"], image_size)
    x_val, y_val_str = build_dataset(splits["val"], image_size)
    x_test, y_test_str = build_dataset(splits["test"], image_size)

    # Encode labels
    label_encoder = LabelEncoder()
    y_train = label_encoder.fit_transform(y_train_str)
    y_val = label_encoder.transform(y_val_str)
    y_test = label_encoder.transform(y_test_str)
    target_names = list(label_encoder.classes_)
    print(f"Found {len(target_names)} classes: {target_names}")

    # Define model and its parameters
    mlp_params = {
        "hidden_layer_sizes": tuple(args.hidden_layer_sizes),
        "activation": args.activation,
        "solver": args.solver,
        "learning_rate_init": args.learning_rate_init,
        "max_iter": args.max_iter,
        "random_state": 42,
        "early_stopping": True,
        "validation_fraction": 0.1,
        "n_iter_no_change": 10,
    }
    mlp = MLPClassifier(**mlp_params)

    # Train and evaluate
    train_and_evaluate(
        name="MLP",
        model=mlp,
        x_train=x_train,
        y_train=y_train,
        x_val=x_val,
        y_val=y_val,
        x_test=x_test,
        y_test=y_test,
        target_names=target_names,
        plots_dir=plots_dir,
        params=mlp_params,
    )
    print(f"\n MLP evaluation complete. Plots saved to '{plots_dir}'.")

    print(f"\nAll trials complete. Plots saved under '{base_plots_dir}'.")


if __name__ == "__main__":
    main()