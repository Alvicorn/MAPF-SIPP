# Base image for MAPF-SIPP project .

FROM python:3.14-slim

# Set environment variables to avoid .pyc files and ensure logs flush immediately
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Install base dependencies
RUN apt-get update -y \
    && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
         apt-utils \
         build-essential \
         ca-certificates \
         lsb-release \
         git \
         ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --break-system-packages \
          pre-commit \
          numpy \
          matplotlib \
          pytest \
          pytest-cov \
          pytest-xdist

# Verify Python version
RUN python3 --version
