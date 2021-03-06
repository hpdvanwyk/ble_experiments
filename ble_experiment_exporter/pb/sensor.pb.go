// Code generated by protoc-gen-go. DO NOT EDIT.
// source: sensor.proto

package pb

import (
	fmt "fmt"
	proto "github.com/golang/protobuf/proto"
	math "math"
)

// Reference imports to suppress errors if they are not otherwise used.
var _ = proto.Marshal
var _ = fmt.Errorf
var _ = math.Inf

// This is a compile-time assertion to ensure that this generated file
// is compatible with the proto package it is being compiled against.
// A compilation error at this line likely means your copy of the
// proto package needs to be updated.
const _ = proto.ProtoPackageIsVersion3 // please upgrade the proto package

type CentralMessage struct {
	Rssi                 int32    `protobuf:"zigzag32,1,opt,name=rssi,proto3" json:"rssi,omitempty"`
	RemoteId             []byte   `protobuf:"bytes,2,opt,name=RemoteId,json=remoteId,proto3" json:"RemoteId,omitempty"`
	XXX_NoUnkeyedLiteral struct{} `json:"-"`
	XXX_unrecognized     []byte   `json:"-"`
	XXX_sizecache        int32    `json:"-"`
}

func (m *CentralMessage) Reset()         { *m = CentralMessage{} }
func (m *CentralMessage) String() string { return proto.CompactTextString(m) }
func (*CentralMessage) ProtoMessage()    {}
func (*CentralMessage) Descriptor() ([]byte, []int) {
	return fileDescriptor_c749425f02243e2d, []int{0}
}

func (m *CentralMessage) XXX_Unmarshal(b []byte) error {
	return xxx_messageInfo_CentralMessage.Unmarshal(m, b)
}
func (m *CentralMessage) XXX_Marshal(b []byte, deterministic bool) ([]byte, error) {
	return xxx_messageInfo_CentralMessage.Marshal(b, m, deterministic)
}
func (m *CentralMessage) XXX_Merge(src proto.Message) {
	xxx_messageInfo_CentralMessage.Merge(m, src)
}
func (m *CentralMessage) XXX_Size() int {
	return xxx_messageInfo_CentralMessage.Size(m)
}
func (m *CentralMessage) XXX_DiscardUnknown() {
	xxx_messageInfo_CentralMessage.DiscardUnknown(m)
}

var xxx_messageInfo_CentralMessage proto.InternalMessageInfo

func (m *CentralMessage) GetRssi() int32 {
	if m != nil {
		return m.Rssi
	}
	return 0
}

func (m *CentralMessage) GetRemoteId() []byte {
	if m != nil {
		return m.RemoteId
	}
	return nil
}

type SensorMessage struct {
	Readings             []*Reading      `protobuf:"bytes,1,rep,name=Readings,json=readings,proto3" json:"Readings,omitempty"`
	ProxyCentral         *CentralMessage `protobuf:"bytes,2,opt,name=ProxyCentral,json=proxyCentral,proto3" json:"ProxyCentral,omitempty"`
	Rssi                 int32           `protobuf:"zigzag32,3,opt,name=rssi,proto3" json:"rssi,omitempty"`
	BatteryVoltage       int32           `protobuf:"varint,4,opt,name=BatteryVoltage,json=batteryVoltage,proto3" json:"BatteryVoltage,omitempty"`
	XXX_NoUnkeyedLiteral struct{}        `json:"-"`
	XXX_unrecognized     []byte          `json:"-"`
	XXX_sizecache        int32           `json:"-"`
}

func (m *SensorMessage) Reset()         { *m = SensorMessage{} }
func (m *SensorMessage) String() string { return proto.CompactTextString(m) }
func (*SensorMessage) ProtoMessage()    {}
func (*SensorMessage) Descriptor() ([]byte, []int) {
	return fileDescriptor_c749425f02243e2d, []int{1}
}

func (m *SensorMessage) XXX_Unmarshal(b []byte) error {
	return xxx_messageInfo_SensorMessage.Unmarshal(m, b)
}
func (m *SensorMessage) XXX_Marshal(b []byte, deterministic bool) ([]byte, error) {
	return xxx_messageInfo_SensorMessage.Marshal(b, m, deterministic)
}
func (m *SensorMessage) XXX_Merge(src proto.Message) {
	xxx_messageInfo_SensorMessage.Merge(m, src)
}
func (m *SensorMessage) XXX_Size() int {
	return xxx_messageInfo_SensorMessage.Size(m)
}
func (m *SensorMessage) XXX_DiscardUnknown() {
	xxx_messageInfo_SensorMessage.DiscardUnknown(m)
}

var xxx_messageInfo_SensorMessage proto.InternalMessageInfo

func (m *SensorMessage) GetReadings() []*Reading {
	if m != nil {
		return m.Readings
	}
	return nil
}

func (m *SensorMessage) GetProxyCentral() *CentralMessage {
	if m != nil {
		return m.ProxyCentral
	}
	return nil
}

func (m *SensorMessage) GetRssi() int32 {
	if m != nil {
		return m.Rssi
	}
	return 0
}

func (m *SensorMessage) GetBatteryVoltage() int32 {
	if m != nil {
		return m.BatteryVoltage
	}
	return 0
}

type Reading struct {
	Id                   []byte   `protobuf:"bytes,1,opt,name=Id,json=id,proto3" json:"Id,omitempty"`
	Temperature          int32    `protobuf:"zigzag32,2,opt,name=Temperature,json=temperature,proto3" json:"Temperature,omitempty"`
	Humidity             int32    `protobuf:"varint,3,opt,name=Humidity,json=humidity,proto3" json:"Humidity,omitempty"`
	Battery              int32    `protobuf:"varint,4,opt,name=Battery,json=battery,proto3" json:"Battery,omitempty"`
	Current              float32  `protobuf:"fixed32,5,opt,name=Current,json=current,proto3" json:"Current,omitempty"`
	XXX_NoUnkeyedLiteral struct{} `json:"-"`
	XXX_unrecognized     []byte   `json:"-"`
	XXX_sizecache        int32    `json:"-"`
}

func (m *Reading) Reset()         { *m = Reading{} }
func (m *Reading) String() string { return proto.CompactTextString(m) }
func (*Reading) ProtoMessage()    {}
func (*Reading) Descriptor() ([]byte, []int) {
	return fileDescriptor_c749425f02243e2d, []int{2}
}

func (m *Reading) XXX_Unmarshal(b []byte) error {
	return xxx_messageInfo_Reading.Unmarshal(m, b)
}
func (m *Reading) XXX_Marshal(b []byte, deterministic bool) ([]byte, error) {
	return xxx_messageInfo_Reading.Marshal(b, m, deterministic)
}
func (m *Reading) XXX_Merge(src proto.Message) {
	xxx_messageInfo_Reading.Merge(m, src)
}
func (m *Reading) XXX_Size() int {
	return xxx_messageInfo_Reading.Size(m)
}
func (m *Reading) XXX_DiscardUnknown() {
	xxx_messageInfo_Reading.DiscardUnknown(m)
}

var xxx_messageInfo_Reading proto.InternalMessageInfo

func (m *Reading) GetId() []byte {
	if m != nil {
		return m.Id
	}
	return nil
}

func (m *Reading) GetTemperature() int32 {
	if m != nil {
		return m.Temperature
	}
	return 0
}

func (m *Reading) GetHumidity() int32 {
	if m != nil {
		return m.Humidity
	}
	return 0
}

func (m *Reading) GetBattery() int32 {
	if m != nil {
		return m.Battery
	}
	return 0
}

func (m *Reading) GetCurrent() float32 {
	if m != nil {
		return m.Current
	}
	return 0
}

func init() {
	proto.RegisterType((*CentralMessage)(nil), "CentralMessage")
	proto.RegisterType((*SensorMessage)(nil), "SensorMessage")
	proto.RegisterType((*Reading)(nil), "Reading")
}

func init() { proto.RegisterFile("sensor.proto", fileDescriptor_c749425f02243e2d) }

var fileDescriptor_c749425f02243e2d = []byte{
	// 312 bytes of a gzipped FileDescriptorProto
	0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xff, 0x54, 0x91, 0x4f, 0x4b, 0xc3, 0x30,
	0x18, 0xc6, 0x4d, 0xb6, 0xae, 0xe1, 0x6d, 0x9d, 0x2e, 0x20, 0x94, 0x9d, 0xca, 0x0e, 0x52, 0x10,
	0x76, 0x98, 0x37, 0x2f, 0x83, 0xee, 0xe2, 0x40, 0x41, 0xa2, 0x78, 0xf0, 0x64, 0x6a, 0x5f, 0x66,
	0x61, 0x4b, 0x4a, 0x92, 0xc1, 0xf6, 0x35, 0xfa, 0x59, 0xfc, 0x80, 0xb2, 0x2c, 0xfe, 0xd9, 0x2d,
	0xcf, 0xf3, 0xbe, 0x4f, 0x9e, 0x5f, 0x08, 0xa4, 0x16, 0x95, 0xd5, 0x66, 0xda, 0x1a, 0xed, 0xf4,
	0x38, 0x55, 0x52, 0xe9, 0xb6, 0x3a, 0xaa, 0xc9, 0x03, 0x0c, 0x17, 0xa8, 0x9c, 0x91, 0xeb, 0x47,
	0xb4, 0x56, 0xae, 0x90, 0x73, 0xe8, 0x1b, 0x6b, 0x9b, 0x8c, 0xe4, 0xa4, 0x18, 0x09, 0x7f, 0xe6,
	0xd7, 0xc0, 0x04, 0x6e, 0xb4, 0xc3, 0x65, 0x9d, 0xd1, 0x9c, 0x14, 0x69, 0x09, 0xdd, 0x9c, 0xb2,
	0x41, 0x37, 0xa7, 0x3b, 0x22, 0x98, 0x09, 0xb3, 0xc9, 0x17, 0x81, 0xf3, 0x67, 0x5f, 0xf6, 0x73,
	0xdb, 0xcd, 0x21, 0x29, 0xeb, 0x46, 0xad, 0x6c, 0x46, 0xf2, 0x5e, 0x91, 0xcc, 0xd8, 0x34, 0x18,
	0x65, 0xd4, 0xcd, 0xe9, 0x65, 0x74, 0x88, 0x1f, 0x17, 0xf8, 0x1d, 0xa4, 0x4f, 0x46, 0xef, 0xf6,
	0x81, 0xc8, 0x57, 0x25, 0xb3, 0x8b, 0xe9, 0x29, 0xa1, 0xcf, 0xbd, 0x9f, 0x89, 0xb4, 0xfd, 0xb7,
	0xfb, 0x8b, 0xdd, 0x3b, 0xc1, 0x1e, 0x96, 0xd2, 0x39, 0x34, 0xfb, 0x57, 0xbd, 0x76, 0x72, 0x85,
	0x59, 0x3f, 0x27, 0x45, 0x24, 0x86, 0xd5, 0x89, 0x3b, 0xe9, 0x08, 0xc4, 0x01, 0x8a, 0x5f, 0x01,
	0x5d, 0xd6, 0xfe, 0xf1, 0xa9, 0x2f, 0x62, 0x03, 0x41, 0x9b, 0x9a, 0xe7, 0x90, 0xbc, 0xe0, 0xa6,
	0x45, 0x23, 0xdd, 0xd6, 0xa0, 0x27, 0x1b, 0x89, 0xc4, 0xfd, 0x59, 0x7c, 0x0c, 0xec, 0x7e, 0xbb,
	0x69, 0xea, 0xc6, 0xed, 0x3d, 0x44, 0x24, 0xd8, 0x67, 0xd0, 0x3c, 0x83, 0x38, 0x80, 0x04, 0x82,
	0x38, 0x10, 0x1c, 0x26, 0x8b, 0xad, 0x31, 0xa8, 0x5c, 0x16, 0xe5, 0xa4, 0xa0, 0x22, 0xfe, 0x38,
	0xca, 0xb2, 0xff, 0x46, 0xdb, 0xaa, 0x1a, 0xf8, 0x6f, 0xba, 0xfd, 0x0e, 0x00, 0x00, 0xff, 0xff,
	0x40, 0xd1, 0x6d, 0xdb, 0xc4, 0x01, 0x00, 0x00,
}
