FROM golang:1.26 AS builder
WORKDIR /usr/src/app
COPY . .
# Static build required so that we can safely copy the binary over.
# `-tags timetzdata` embeds zone info from the "time/tzdata" package.
RUN CGO_ENABLED=0 GOBIN=$(pwd)/bin go install -mod=vendor .

FROM scratch
# the test program:
COPY --from=builder /usr/src/app/bin/ble_experiments_exporter /ble_experiments_exporter
ENTRYPOINT ["/ble_experiments_exporter"]