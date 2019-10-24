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
	"ble_experiment_exporter/pb"
	"log"

	gogoio "github.com/gogo/protobuf/io"
	"go.bug.st/serial.v1"
)

type SensorReader struct {
	tty     string
	msgChan chan *Messages
}

func NewSensorReader(tty string, msgChan chan *Messages) *SensorReader {
	s := &SensorReader{
		tty:     tty,
		msgChan: msgChan,
	}
	return s
}

func (s *SensorReader) ReadLoop() {
	mode := &serial.Mode{
		BaudRate: 115200,
	}
	port, err := serial.Open(s.tty, mode)
	if err != nil {
		log.Print(err)
		return
	}
	delimReader := gogoio.NewDelimitedReader(port, 2048)
	defer delimReader.Close()
	for {
		msg, err := readMessages(delimReader)
		if err != nil {
			log.Print(err)
			return
		}
		s.msgChan <- msg
	}
}

type Messages struct {
	Central *pb.CentralMessage
	Sensor  *pb.SensorMessage
}

func readMessages(r gogoio.ReadCloser) (*Messages, error) {
	centralMsg := pb.CentralMessage{}
	err := r.ReadMsg(&centralMsg)
	if err != nil {
		return nil, err
	}
	sensorMsg := pb.SensorMessage{}
	err = r.ReadMsg(&sensorMsg)
	if err != nil {
		return nil, err
	}
	return &Messages{
		Central: &centralMsg,
		Sensor:  &sensorMsg,
	}, nil
}
