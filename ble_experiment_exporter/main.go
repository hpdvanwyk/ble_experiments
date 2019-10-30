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
	"flag"
	"io/ioutil"
	"log"
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"gopkg.in/yaml.v2"
)

func main() {
	ttyDev := flag.String("tty", "/dev/ttyACM0", "the tty device to use")
	addr := flag.String("addr", ":8080", "TCP address to listen on.")
	config := flag.String("config", "", "Config file")

	parsedConfig := &SensorConfig{}
	flag.Parse()
	if *config != "" {
		data, err := ioutil.ReadFile(*config)
		if err != nil {
			log.Fatalf("error: %v", err)
		}
		err = yaml.Unmarshal(data, parsedConfig)
		if err != nil {
			log.Fatalf("error: %v", err)
		}
	}
	http.Handle("/metrics", promhttp.Handler())
	go func() {
		log.Fatal(http.ListenAndServe(*addr, nil))
	}()
	se := NewSensorExporter(parsedConfig)
	go se.Run()
	prometheus.MustRegister(se)
	sr := NewSensorReader(*ttyDev, se.MsgChan)
	for {
		sr.ReadLoop()
		time.Sleep(5 * time.Second)
	}
}
