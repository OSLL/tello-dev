FROM python

RUN apt-get update && \
    apt-get install -y python3-pip ffmpeg libsm6 libxext6

COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /app

COPY . /app

CMD ["python", "run_singel_drone.py"]
