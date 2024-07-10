FROM ubuntu:jammy
ADD drake-latest-jammy.tar.gz /opt
WORKDIR ./dairlib
ENV DAIRLIB_DOCKER_VERSION_25=25
COPY . .
RUN export DEBIAN_FRONTEND=noninteractive \
    && apt-get update -qq \
    && apt-get install --no-install-recommends -qy \
     $(cat /opt/drake/share/drake/setup/packages-jammy.txt) \
     python3-pip python-is-python3 \
      && rm -rf /var/lib/apt/lists/* \
#RUN set -eux \
#  && apt-get install --no-install-recommends  locales \
#  && locale-gen en_US.UTF-8
#RUN if type sudo 2>/dev/null; then \
#     echo "The sudo command already exists... Skipping."; \
#    else \
#     echo -e "#!/bin/sh\n\${@}" > /usr/sbin/sudo; \
#     chmod +x /usr/sbin/sudo; \
#    fi
#RUN set -eux \
#  && export DEBIAN_FRONTEND=noninteractive \
#  && yes | install/install_prereqs_jammy.sh \
#  && rm -rf /var/lib/apt/lists/*