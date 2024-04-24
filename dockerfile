FROM osrf/ros2

WORKDIR /src

COPY . .

RUN ["python3", "main.py"]