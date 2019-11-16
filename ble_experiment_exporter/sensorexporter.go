/*
Copyright (c) 2019 Hendrik van Wyk
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package main

import (
	"fmt"
	"net"
	"sync"
	"time"

	"ble_experiment_exporter/pb"

	"github.com/prometheus/client_golang/prometheus"
)

const DefaultExpiryTime = 100 * time.Second

type SensorConfig struct {
	Rename  map[string]string
	Fudge   map[string]float64
	Timeout map[string]int
}

func IdString(id []byte) string {
	return net.HardwareAddr(id).String()
}

type label struct {
	key   string
	value string
}

type metric struct {
	value       float64
	lables      []*label
	opts        *prometheus.GaugeOpts
	lastUpdated time.Time
	expiryTime  time.Duration
}

func (s *SensorExporter) IdString(id []byte) string {
	idString := IdString(id)
	rename, hasRename := s.config.Rename[idString]
	if hasRename {
		return rename
	}
	return idString
}

func (s *SensorExporter) fudgeTemperature(id []byte) float64 {
	idString := IdString(id)
	fudge, hasFudge := s.config.Fudge[idString]
	if hasFudge {
		return fudge
	}
	return 0
}

func (s *SensorExporter) expiryTime(id []byte) time.Duration {
	idString := IdString(id)
	timeout, hasTimeout := s.config.Timeout[idString]
	if hasTimeout {
		return time.Duration(timeout) * time.Second
	}
	return DefaultExpiryTime
}

func (se *SensorExporter) updateMetric(value float64,
	opts *prometheus.GaugeOpts,
	expiryTime time.Duration,
	labels ...*label) {
	se.Lock()
	defer se.Unlock()
	k := opts.Name + "~"
	for i := range labels {
		k += labels[i].key + "~" + labels[i].value + "~"
	}
	t, ok := se.metrics[k]
	if !ok {
		t = &metric{
			lables:     labels,
			opts:       opts,
			expiryTime: expiryTime,
		}
		se.metrics[k] = t
	}
	t.value = value
	t.lastUpdated = time.Now()
}

func (se *SensorExporter) gc() {
	se.Lock()
	defer se.Unlock()
	for i := range se.metrics {
		if time.Since(se.metrics[i].lastUpdated) > se.metrics[i].expiryTime {
			delete(se.metrics, i)
		}
	}
}

type SensorExporter struct {
	sync.RWMutex
	MsgChan chan *Messages
	close   chan struct{}
	metrics map[string]*metric
	config  *SensorConfig
}

var TemperatureOpts = prometheus.GaugeOpts{
	Name: "ble_sensor_temperature_c",
	Help: "Temperature reading from a ble sensor.",
}

var TemperatureFudgeOpts = prometheus.GaugeOpts{
	Name: "ble_sensor_temperature_fudge_c",
	Help: "Temperature reading adjustment.",
}

var RssiOpts = prometheus.GaugeOpts{
	Name: "ble_rssi_db",
	Help: "Rssi of ble device.",
}

var RemoteRssiOpts = prometheus.GaugeOpts{
	Name: "ble_remote_rssi_db",
	Help: "Rssi seen on remote ble device.",
}

var HumidityOpts = prometheus.GaugeOpts{
	Name: "ble_sensor_humidity_p",
	Help: "Humidity.",
}

var BatteryOpts = prometheus.GaugeOpts{
	Name: "ble_battery_p",
	Help: "Battery charge left.",
}

var BatteryVoltageOpts = prometheus.GaugeOpts{
	Name: "ble_battery_voltage_v",
	Help: "Battery voltage.",
}

func NewSensorExporter(config *SensorConfig) *SensorExporter {
	s := &SensorExporter{
		MsgChan: make(chan *Messages, 2),
		close:   make(chan struct{}),
		metrics: make(map[string]*metric),
		config:  config,
	}
	return s
}

func (s *SensorExporter) Describe(chan<- *prometheus.Desc) {
}

func (se *SensorExporter) Collect(metricChan chan<- prometheus.Metric) {
	se.Lock()
	defer se.Unlock()
	for i, m := range se.metrics {
		opts := *se.metrics[i].opts
		labels := make(map[string]string)
		for j := range m.lables {
			labels[m.lables[j].key] = m.lables[j].value
		}
		opts.ConstLabels = labels
		g := prometheus.NewGauge(opts)
		g.Set(m.value)
		metricChan <- g
	}
}

func (s *SensorExporter) exportCentral(msg *Messages) *pb.CentralMessage {
	expiryTime := s.expiryTime(msg.Central.RemoteId)
	central := msg.Central
	s.updateMetric(
		float64(msg.Central.Rssi),
		&RssiOpts,
		expiryTime,
		&label{"remoteid", s.IdString(msg.Central.RemoteId)},
	)

	fmt.Printf("rssi: %v dB\ndevice id: %v\n", msg.Central.Rssi, s.IdString(msg.Central.RemoteId))
	if msg.Sensor.Rssi != 0 {
		s.updateMetric(
			float64(msg.Sensor.Rssi),
			&RemoteRssiOpts,
			expiryTime,
			&label{"remoteid", s.IdString(msg.Central.RemoteId)},
		)
	}

	if msg.Sensor.BatteryVoltage != 0 {
		s.updateMetric(
			float64(msg.Sensor.BatteryVoltage)/100,
			&BatteryVoltageOpts,
			expiryTime*4,
			&label{"remoteid", s.IdString(msg.Central.RemoteId)},
		)
	}

	if msg.Sensor.ProxyCentral != nil {
		central = msg.Sensor.ProxyCentral
		expiryTime := s.expiryTime(central.RemoteId)
		s.updateMetric(
			float64(central.Rssi),
			&RssiOpts,
			expiryTime,
			&label{"remoteid", s.IdString(central.RemoteId)},
			&label{"proxyid", s.IdString(msg.Central.RemoteId)},
		)
	}
	return central
}

func (s *SensorExporter) exportReading(c *pb.CentralMessage, r *pb.Reading) {
	expiryTime := s.expiryTime(c.RemoteId)

	if r.Temperature != 0 {
		fudge := s.fudgeTemperature(r.Id)
		s.updateMetric(
			fudge,
			&TemperatureFudgeOpts,
			expiryTime,
			&label{"remoteid", s.IdString(c.RemoteId)},
			&label{"sensorid", s.IdString(r.Id)},
		)
		s.updateMetric(
			float64(r.Temperature)/10,
			&TemperatureOpts,
			expiryTime,
			&label{"remoteid", s.IdString(c.RemoteId)},
			&label{"sensorid", s.IdString(r.Id)},
		)
	}
	if r.Humidity != 0 {
		s.updateMetric(
			float64(r.Humidity)/10,
			&HumidityOpts,
			expiryTime,
			&label{"remoteid", s.IdString(c.RemoteId)},
			&label{"sensorid", s.IdString(r.Id)},
		)
	}
	if r.Battery != 0 {
		s.updateMetric(
			float64(r.Battery),
			&BatteryOpts,
			expiryTime*4,
			&label{"remoteid", s.IdString(c.RemoteId)},
			&label{"sensorid", s.IdString(r.Id)},
		)
	}
}

func (s *SensorExporter) Run() {
	cleanup := time.NewTicker(1 * time.Second)
	for {
		select {
		case msg := <-s.MsgChan:
			central := s.exportCentral(msg)
			for _, r := range msg.Sensor.Readings {
				s.exportReading(central, r)
			}
		case <-cleanup.C:
			s.gc()
		case <-s.close:
			cleanup.Stop()
			return
		}
	}
}
