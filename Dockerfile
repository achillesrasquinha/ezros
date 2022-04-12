

FROM  python:3.7-alpine

LABEL maintainer=achillesrasquinha@gmail.com

ENV EZROS_PATH=/usr/local/src/ezros

RUN apk add --no-cache \
        bash \
        git \
    && mkdir -p $EZROS_PATH

COPY . $EZROS_PATH
COPY ./docker/entrypoint.sh /entrypoint.sh

WORKDIR $EZROS_PATH

RUN pip install -r ./requirements.txt && \
    python setup.py install

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ezros"]