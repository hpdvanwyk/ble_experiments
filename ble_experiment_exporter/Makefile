build:
	GOBIN=$(shell pwd)/bin go install -mod=vendor .

lint:
	golangci-lint run -v ./...

proto:
	protoc \
	-I ../ble_sensor_coded/pb \
	-I ../ble_sensor_coded/nanopb/generator/proto/ \
	--go_out=pb \
	--go_opt=paths=source_relative \
	--go_opt=Mnanopb.proto=github.com/hpdvanwyk/ble_experiments/ble_experiments_exporter/pb \
	../ble_sensor_coded/pb/sensor.proto nanopb.proto