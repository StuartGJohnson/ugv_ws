# ================================================================
# ROS2-ML Environment Setup
#  - Creates a venv that inherits ROS2 (rclpy, etc.)
#  - Installs PyTorch + CVXPY stack
#  - Choose: make setup-cpu   or   make setup-cu124
# ================================================================

SHELL	 := /bin/bash
VENV_DIR ?= $(PWD)/venv
PYTHON   ?= python3
PIP      := $(VENV_DIR)/bin/pip
ACTIVATE := source $(VENV_DIR)/bin/activate

# ------------------------------------------------
# Generic venv creation target
# ------------------------------------------------
$(VENV_DIR):
	@echo ">>> Creating ROS2-aware venv at $(VENV_DIR)"
	$(PYTHON) -m venv $(VENV_DIR) --system-site-packages --prompt ros2-ml
	$(PIP) install --upgrade pip

# ------------------------------------------------
# CPU setup
# ------------------------------------------------
setup-cpu: $(VENV_DIR)
	@echo ">>> Installing CPU-only stack"
	$(PIP) install -r requirements-ros2-ml-cpu.txt
	@echo
	@echo ">>> Done. Activate with:"
	@echo "    source $(VENV_DIR)/bin/activate"

# ------------------------------------------------
# CUDA 12.4 setup
# ------------------------------------------------
setup-cu124: $(VENV_DIR)
	@echo ">>> Installing CUDA 12.4 stack"
	$(PIP) install -r requirements-ros2-ml-cu124.txt
	@echo
	@echo ">>> Done. Activate with:"
	@echo "    source $(VENV_DIR)/bin/activate"

# ------------------------------------------------
# Utilities
# ------------------------------------------------
activate:
	source $(VENV_DIR)/bin/activate

check:
	@$(ACTIVATE) && \
	python -c 'import torch, cvxpy as cp, rclpy; \
	print("torch:", torch.__version__, "cuda?", torch.cuda.is_available()); \
	print("cvxpy:", cp.__version__); \
	print("rclpy OK")'

clean:
	@echo ">>> Removing venv at $(VENV_DIR)"
	rm -rf $(VENV_DIR)

