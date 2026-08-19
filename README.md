# Upgrading Humanoid Robot Mechatronics Without Replacing Actuators

Analysis code and data for a field-failure-driven redesign methodology, developed over three hardware generations of an adult-size humanoid robot contested in the RoboCup Humanoid League.

Everything in this repository reproduces the analytical results of the accompanying paper from a single script, so that any assumption can be changed and its consequence seen immediately.

---

## Running the analysis

```bash
pip install -r requirements.txt
cd analysis
python reproduce_analysis.py
```

This prints every analytical table in the paper and writes both analysis figures to `analysis/outputs/`.

All model parameters live in one `PARAMETERS` block at the top of `reproduce_analysis.py`. Change any of them and rerun to see the consequence — that is the point of publishing this rather than only the numbers.

---

## Repository layout

```
analysis/
  reproduce_analysis.py     all tables and figures, one file, no hidden state
data/
  failure_log_TEMPLATE.csv  structure for the per-match failure record
docs/
  images/                   figures used in this README
```

