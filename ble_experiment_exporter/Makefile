build:
	GOBIN=$(shell pwd)/bin go install -mod=vendor .

lint:
	golangci-lint run -v ./...

proto:
	cd pb&&protoc -I ../../ble_sensor_coded/pb -I../../ble_sensor_coded/nanopb/generator/proto/ --go_out=. ../../ble_sensor_coded/pb/sensor.proto