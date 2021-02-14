FROM debian

COPY scripts/install_build_deps.sh .

RUN apt-get update
RUN  sh ./install_build_deps.sh

ARG USER
ARG HOME

RUN useradd -ms /bin/bash $USER

USER $USER
WORKDIR $HOME