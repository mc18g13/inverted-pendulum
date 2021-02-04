FROM debian

COPY scripts/install_build_deps.sh .

RUN apt-get update
RUN chmod +x install_build_deps.sh
RUN ./install_build_deps.sh
