FROM nginx:alpine

RUN apk update && \
    apk add --no-cache \
        bash

COPY ./nginx.conf /nginx.conf.template

COPY ./start /start
RUN chmod +x /start

CMD [ "/start" ]