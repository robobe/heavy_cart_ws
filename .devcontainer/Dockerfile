FROM humble:dev

RUN apt-get update && apt-get install -y \
    curl \
    putty \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout user

USER user

RUN python3 -m pip install -U platformio

# RUN /home/user/.local/bin/pio/pio platform install espressif32