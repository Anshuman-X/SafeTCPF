"""
Standalone script to re-run only the visualization and PDF-report steps
using the already-generated raw_experiment_results.csv.
Run from the SafeTCPF workspace root.
"""
import os
import sys
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from evaluation.experiment_runner import ExperimentRunner

runner = ExperimentRunner()

# Load pre-existing raw results
raw_csv = os.path.join(runner.results_dir, "raw_experiment_results.csv")
print(f"Loading {raw_csv} ...")
raw_df = pd.read_csv(raw_csv)
print(f"Loaded {len(raw_df)} rows.")

# Regenerate figures
print("Generating visualizations...")
runner.generate_visualizations(raw_df)

# Regenerate PDF reports
print("Generating PDF reports...")
runner.generate_pdf_reports(raw_df)

print("\nDone! Check figures/ and reports/ directories.")
