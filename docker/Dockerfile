# © 2025 ETH Zurich, Robotic Systems Lab
# Licensed under the Apache License 2.0
#
# PACE on top of the official Isaac Lab image (Isaac Lab 2.3.2 / Isaac Sim 5.1).

ARG ISAACLAB_IMAGE=nvcr.io/nvidia/isaac-lab:2.3.2
FROM ${ISAACLAB_IMAGE}

ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    PACE_PATH=/workspace/pace-sim2real

# Expose the Isaac Sim interpreter as `python` so the README commands work verbatim
RUN printf '#!/bin/bash\nexec /isaac-sim/python.sh "$@"\n' > /usr/local/bin/python \
    && chmod +x /usr/local/bin/python

# Install PACE in editable mode. The repository is bind-mounted over ${PACE_PATH}
# at runtime, so local code changes are picked up without rebuilding.
COPY source/pace_sim2real ${PACE_PATH}/source/pace_sim2real
RUN python -m pip install --no-cache-dir -e ${PACE_PATH}/source/pace_sim2real

WORKDIR ${PACE_PATH}
ENTRYPOINT []
CMD ["/bin/bash"]
