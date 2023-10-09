FROM ubuntu:jammy
WORKDIR ./dairlib
ENV DAIRLIB_DOCKER_VERSION_25=25
COPY . .
RUN apt-get update && apt-get install -y wget lsb-release pkg-config zip g++ zlib1g-dev unzip clang-12 clang-format-12 ca-certificates gnupg

RUN set -eux \
  && apt-get install --no-install-recommends  locales \
  && locale-gen en_US.UTF-8
RUN if type sudo 2>/dev/null; then \ 
     echo "The sudo command already exists... Skipping."; \
    else \
     echo -e "#!/bin/sh\n\${@}" > /usr/sbin/sudo; \
     chmod +x /usr/sbin/sudo; \
    fi
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && yes | install/install_prereqs_jammy.sh \
  && rm -rf /var/lib/apt/lists/*
