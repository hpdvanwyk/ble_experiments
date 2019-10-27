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
	"sync"
	"time"

	"github.com/prometheus/client_golang/prometheus"
)

func IdString(id []byte) string {
	if len(id) != 6 {
		// don't do that
		return ""
	}
	return fmt.Sprintf("%02x:%02x:%02x:%02x:%02x:%02x", id[0], id[1], id[2], id[3], id[4], id[5])
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
	MsgChan    chan *Messages
	close      chan struct{}
	metrics    map[string]*metric
	expiryTime time.Duration
}

var TemperatureOpts = prometheus.GaugeOpts{
	Name: "ble_sensor_temperature_c",
	Help: "Temperature reading from a ble sensor.",
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

func NewSensorExporter(expiryTime time.Duration) *SensorExporter {
	s := &SensorExporter{
		MsgChan:    make(chan *Messages, 2),
		close:      make(chan struct{}),
		metrics:    make(map[string]*metric),
		expiryTime: expiryTime,
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

func (s *SensorExporter) Run() {
	cleanup := time.NewTicker(1 * time.Second)
	for {
		select {
		case msg := <-s.MsgChan:
			central := msg.Central
			s.updateMetric(
				float64(msg.Central.Rssi),
				&RssiOpts,
				s.expiryTime,
				&label{"remoteid", IdString(msg.Central.RemoteId)},
			)

			fmt.Printf("rssi: %v dB\ndevice id: %v\n", msg.Central.Rssi, IdString(msg.Central.RemoteId))
			if msg.Sensor.Rssi != 0 {
				s.updateMetric(
					float64(msg.Sensor.Rssi),
					&RemoteRssiOpts,
					s.expiryTime,
					&label{"remoteid", IdString(msg.Central.RemoteId)},
				)
			}

			if msg.Sensor.ProxyCentral != nil {
				central = msg.Sensor.ProxyCentral
				s.updateMetric(
					float64(central.Rssi),
					&RssiOpts,
					s.expiryTime,
					&label{"remoteid", IdString(central.RemoteId)},
					&label{"proxyid", IdString(msg.Central.RemoteId)},
				)
			}
			for i := range msg.Sensor.Readings {
				fmt.Printf("id: %v\nTemperature: %v C\n",
					IdString(msg.Sensor.Readings[i].Id),
					float32(msg.Sensor.Readings[i].Temperature)/10)

				s.updateMetric(
					float64(msg.Sensor.Readings[i].Temperature)/10,
					&TemperatureOpts,
					s.expiryTime,
					&label{"remoteid", IdString(central.RemoteId)},
					&label{"sensorid", IdString(msg.Sensor.Readings[i].Id)},
				)
			}
			if msg.Sensor.MJReading != nil {
				r := msg.Sensor.MJReading
				if r.Temperature != 0 {
					s.updateMetric(
						float64(r.Temperature)/10,
						&TemperatureOpts,
						1*time.Minute,
						&label{"remoteid", IdString(central.RemoteId)},
						&label{"sensorid", "0"},
					)
				}
				if r.Humidity != 0 {
					s.updateMetric(
						float64(r.Humidity)/10,
						&HumidityOpts,
						1*time.Minute,
						&label{"remoteid", IdString(central.RemoteId)},
						&label{"sensorid", "0"},
					)
				}
				if r.Battery != 0 {
					s.updateMetric(
						float64(r.Battery),
						&BatteryOpts,
						5*time.Minute,
						&label{"remoteid", IdString(central.RemoteId)},
					)
				}
			}
		case <-cleanup.C:
			s.gc()
		case <-s.close:
			cleanup.Stop()
			return
		}
	}
}
