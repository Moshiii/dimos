# Development image for dimos on the Galaxea R1 Lite.
# Build with docker/galaxea-r1lite/build.sh (refuses a dirty tree, pins
# the platform, stamps OCI labels). Not a production deployment artifact.

# Bump the base deliberately:
#   docker image inspect ros:humble-ros-base-jammy --format '{{index .RepoDigests 0}}'
ARG ROS_BASE=ros@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16
ARG UV_VERSION=0.11.17

FROM ${ROS_BASE} AS lockexport
ARG UV_VERSION
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir "uv==${UV_VERSION}"
COPY pyproject.toml uv.lock /src/
# pygame (sim extra), the Pink solver runtime (manipulation extra) and
# the websocket-vis runtime are exported separately: mixing unhashed
# extra pins into the hashed main export fails the install. The Pink
# subset is exactly what the teleop solver needs beyond the main closure
# (pinocchio is already in it). The vis/web subset exists because
# vis_module imports socketio and the quest server imports fastapi
# unconditionally, while those ship only in dev groups or extras;
# versions still come from the lockfile.
RUN cd /src && uv export --frozen --no-emit-project --no-default-groups \
        --format requirements-txt -o /requirements.txt \
    && test -s /requirements.txt \
    && grep -q '^typer==' /requirements.txt \
    && ! grep -qE '^(torch|mujoco|pyaudio)==' /requirements.txt \
    && uv export --frozen --no-emit-project --no-default-groups --extra sim \
        --format requirements-txt \
        | grep -E '^pygame==' | cut -d' ' -f1 > /pygame.txt \
    && [ "$(grep -c '^pygame==' /pygame.txt)" = "1" ] \
    && uv export --frozen --no-emit-project --no-default-groups --extra manipulation \
        --format requirements-txt \
        | grep -E '^(pin-pink|qpsolvers|proxsuite|loop-rate-limiters)==' \
        | cut -d' ' -f1 > /pink.txt \
    && [ "$(wc -l < /pink.txt)" = "4" ] \
    && uv export --frozen --no-emit-project --no-default-groups --group lint \
        --format requirements-txt \
        | grep -E '^(python-socketio|bidict|python-engineio|simple-websocket|wsproto|h11|starlette|uvicorn|anyio|sniffio|exceptiongroup)==' \
        | cut -d' ' -f1 > /vis.txt \
    && uv export --frozen --no-emit-project --no-default-groups --extra web \
        --format requirements-txt \
        | grep -E '^(fastapi|ffmpeg-python|sse-starlette|soundfile|future)==' \
        | cut -d' ' -f1 >> /vis.txt \
    && [ "$(wc -l < /vis.txt)" = "16" ]

FROM ${ROS_BASE} AS builder
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential python3-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*
ENV CIBUILDWHEEL=1
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir "setuptools>=70" "packaging>=24" wheel "pybind11>=2.12" tomli
COPY . /src
RUN pip wheel --no-deps --no-build-isolation --wheel-dir /wheels /src \
    && test -n "$(find /wheels -name 'dimos-*.whl' -print -quit)"

FROM ${ROS_BASE}
ARG UV_VERSION
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python-is-python3 \
        libturbojpeg0-dev \
        liblcm-dev \
        libgl1 \
        libglib2.0-0 \
        libsndfile1 \
        iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Dependency layer, keyed on the lockfile via bind mount. --no-deps: each
# export is a complete closure slice and re-resolving breaks on
# override-dependencies.
RUN --mount=type=bind,from=lockexport,source=/requirements.txt,target=/tmp/requirements.txt \
    --mount=type=bind,from=lockexport,source=/pygame.txt,target=/tmp/pygame.txt \
    --mount=type=bind,from=lockexport,source=/pink.txt,target=/tmp/pink.txt \
    pip install --no-cache-dir "uv==${UV_VERSION}" \
    && uv pip install --system --no-cache --no-deps -r /tmp/requirements.txt \
    && uv pip install --system --no-cache --no-deps -r /tmp/pygame.txt \
    && uv pip install --system --no-cache --no-deps -r /tmp/pink.txt \
    && pip uninstall -y uv

# Separate small layer: the vis subset iterates independently of the big
# dependency layer above.
RUN --mount=type=bind,from=lockexport,source=/vis.txt,target=/tmp/vis.txt \
    pip install --no-cache-dir --no-deps -r /tmp/vis.txt

RUN --mount=type=bind,from=builder,source=/wheels,target=/tmp/wheels \
    pip install --no-cache-dir --no-deps /tmp/wheels/dimos-*.whl

COPY docker/galaxea-r1lite/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# uid 1000 matches the vendor stack so shared-memory DDS delivery works.
RUN useradd --uid 1000 --user-group --create-home --shell /bin/bash dimos
USER dimos
ENV HOME=/home/dimos
ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=2

# Build-time gate, run as the runtime user. Constructing a teleop task
# builds a real PinkStepper, which imports pink and verifies the QP
# backend; a missing or downgraded solver fails the build here instead of
# at robot startup.
RUN dimos list | grep -q r1lite-coordinator \
    && dimos list | grep -q r1lite-quest-teleop \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import r1lite_coordinator" \
    && python3 -c "from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_keyboard_teleop import r1lite_keyboard_teleop" \
    && python3 -c "import pink, proxsuite" \
    && python3 -c "\
from dimos.control.tasks.teleop_task.teleop_task import create_task; \
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import _teleop_tasks; \
tasks = [create_task(c, None) for c in _teleop_tasks()]; \
assert all(t._config.solver == 'pink' for t in tasks)"

# The validated stop path is the SIGINT sequence (the connection streams
# bounded chassis zeros and reports settling); docker stop must trigger
# exactly that.
STOPSIGNAL SIGINT

ENTRYPOINT ["/entrypoint.sh"]
CMD ["run", "r1lite-coordinator"]
