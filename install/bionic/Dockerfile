FROM ubuntu
WORKDIR /dairlib
COPY . .
RUN apt update && apt install -y wget lsb-release pkg-config zip g++ zlib1g-dev unzip python
RUN set -eux \
  && apt-get install --no-install-recommends  locales \
  && locale-gen en_US.UTF-8
RUN if type sudo 2>/dev/null; then \ 
     echo "The sudo command already exists... Skipping."; \
    else \
     echo -e "#!/bin/sh\n\${@}" > /usr/sbin/sudo; \
     chmod +x /usr/sbin/sudo; \
    fi
RUN  set -eux \
  && wget https://github.com/bazelbuild/bazel/releases/download/0.19.2/bazel-0.19.2-installer-linux-x86_64.sh  \
  && chmod +x bazel-0.19.2-installer-linux-x86_64.sh \
  && ./bazel-0.19.2-installer-linux-x86_64.sh 
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && yes | install/install_prereqs_ubuntu.sh \
  && rm -rf /var/lib/apt/lists/*