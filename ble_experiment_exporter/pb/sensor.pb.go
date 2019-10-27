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
	Readings             []*TemperatureReading `protobuf:"bytes,1,rep,name=Readings,json=readings,proto3" json:"Readings,omitempty"`
	MJReading            *MJReading            `protobuf:"bytes,2,opt,name=MJReading,json=mJReading,proto3" json:"MJReading,omitempty"`
	ProxyCentral         *CentralMessage       `protobuf:"bytes,3,opt,name=ProxyCentral,json=proxyCentral,proto3" json:"ProxyCentral,omitempty"`
	Rssi                 int32                 `protobuf:"zigzag32,4,opt,name=rssi,proto3" json:"rssi,omitempty"`
	XXX_NoUnkeyedLiteral struct{}              `json:"-"`
	XXX_unrecognized     []byte                `json:"-"`
	XXX_sizecache        int32                 `json:"-"`
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

func (m *SensorMessage) GetReadings() []*TemperatureReading {
	if m != nil {
		return m.Readings
	}
	return nil
}

func (m *SensorMessage) GetMJReading() *MJReading {
	if m != nil {
		return m.MJReading
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

type TemperatureReading struct {
	Temperature          int32    `protobuf:"zigzag32,1,opt,name=Temperature,json=temperature,proto3" json:"Temperature,omitempty"`
	Id                   []byte   `protobuf:"bytes,2,opt,name=Id,json=id,proto3" json:"Id,omitempty"`
	XXX_NoUnkeyedLiteral struct{} `json:"-"`
	XXX_unrecognized     []byte   `json:"-"`
	XXX_sizecache        int32    `json:"-"`
}

func (m *TemperatureReading) Reset()         { *m = TemperatureReading{} }
func (m *TemperatureReading) String() string { return proto.CompactTextString(m) }
func (*TemperatureReading) ProtoMessage()    {}
func (*TemperatureReading) Descriptor() ([]byte, []int) {
	return fileDescriptor_c749425f02243e2d, []int{2}
}

func (m *TemperatureReading) XXX_Unmarshal(b []byte) error {
	return xxx_messageInfo_TemperatureReading.Unmarshal(m, b)
}
func (m *TemperatureReading) XXX_Marshal(b []byte, deterministic bool) ([]byte, error) {
	return xxx_messageInfo_TemperatureReading.Marshal(b, m, deterministic)
}
func (m *TemperatureReading) XXX_Merge(src proto.Message) {
	xxx_messageInfo_TemperatureReading.Merge(m, src)
}
func (m *TemperatureReading) XXX_Size() int {
	return xxx_messageInfo_TemperatureReading.Size(m)
}
func (m *TemperatureReading) XXX_DiscardUnknown() {
	xxx_messageInfo_TemperatureReading.DiscardUnknown(m)
}

var xxx_messageInfo_TemperatureReading proto.InternalMessageInfo

func (m *TemperatureReading) GetTemperature() int32 {
	if m != nil {
		return m.Temperature
	}
	return 0
}

func (m *TemperatureReading) GetId() []byte {
	if m != nil {
		return m.Id
	}
	return nil
}

type MJReading struct {
	Temperature          int32    `protobuf:"varint,1,opt,name=Temperature,json=temperature,proto3" json:"Temperature,omitempty"`
	Humidity             int32    `protobuf:"varint,2,opt,name=Humidity,json=humidity,proto3" json:"Humidity,omitempty"`
	Battery              int32    `protobuf:"varint,3,opt,name=Battery,json=battery,proto3" json:"Battery,omitempty"`
	XXX_NoUnkeyedLiteral struct{} `json:"-"`
	XXX_unrecognized     []byte   `json:"-"`
	XXX_sizecache        int32    `json:"-"`
}

func (m *MJReading) Reset()         { *m = MJReading{} }
func (m *MJReading) String() string { return proto.CompactTextString(m) }
func (*MJReading) ProtoMessage()    {}
func (*MJReading) Descriptor() ([]byte, []int) {
	return fileDescriptor_c749425f02243e2d, []int{3}
}

func (m *MJReading) XXX_Unmarshal(b []byte) error {
	return xxx_messageInfo_MJReading.Unmarshal(m, b)
}
func (m *MJReading) XXX_Marshal(b []byte, deterministic bool) ([]byte, error) {
	return xxx_messageInfo_MJReading.Marshal(b, m, deterministic)
}
func (m *MJReading) XXX_Merge(src proto.Message) {
	xxx_messageInfo_MJReading.Merge(m, src)
}
func (m *MJReading) XXX_Size() int {
	return xxx_messageInfo_MJReading.Size(m)
}
func (m *MJReading) XXX_DiscardUnknown() {
	xxx_messageInfo_MJReading.DiscardUnknown(m)
}

var xxx_messageInfo_MJReading proto.InternalMessageInfo

func (m *MJReading) GetTemperature() int32 {
	if m != nil {
		return m.Temperature
	}
	return 0
}

func (m *MJReading) GetHumidity() int32 {
	if m != nil {
		return m.Humidity
	}
	return 0
}

func (m *MJReading) GetBattery() int32 {
	if m != nil {
		return m.Battery
	}
	return 0
}

func init() {
	proto.RegisterType((*CentralMessage)(nil), "CentralMessage")
	proto.RegisterType((*SensorMessage)(nil), "SensorMessage")
	proto.RegisterType((*TemperatureReading)(nil), "TemperatureReading")
	proto.RegisterType((*MJReading)(nil), "MJReading")
}

func init() { proto.RegisterFile("sensor.proto", fileDescriptor_c749425f02243e2d) }

var fileDescriptor_c749425f02243e2d = []byte{
	// 304 bytes of a gzipped FileDescriptorProto
	0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xff, 0x6c, 0x91, 0xbd, 0x4e, 0xfb, 0x30,
	0x14, 0xc5, 0xff, 0x76, 0x9b, 0x36, 0xbd, 0xc9, 0x9f, 0x0f, 0xb3, 0x44, 0x9d, 0xa2, 0x0c, 0x28,
	0x53, 0x86, 0x22, 0x31, 0xb0, 0x44, 0x0a, 0x0b, 0x20, 0x2a, 0x21, 0xc3, 0xc4, 0x84, 0x43, 0xac,
	0x12, 0x89, 0xc4, 0x91, 0xed, 0x4a, 0xcd, 0x6b, 0xe4, 0x8d, 0x78, 0x33, 0x54, 0xc7, 0x6d, 0x03,
	0x74, 0xbb, 0x1f, 0x27, 0xe7, 0xe4, 0x77, 0x0d, 0xbe, 0xe2, 0xb5, 0x12, 0x32, 0x69, 0xa4, 0xd0,
	0x62, 0xee, 0xd7, 0xac, 0x16, 0x4d, 0xde, 0x77, 0xd1, 0x23, 0x9c, 0xdc, 0xf2, 0x5a, 0x4b, 0xf6,
	0xb9, 0xe4, 0x4a, 0xb1, 0x15, 0x27, 0x04, 0xc6, 0x52, 0xa9, 0x32, 0x40, 0x21, 0x8a, 0xcf, 0xa9,
	0xa9, 0xc9, 0x25, 0xb8, 0x94, 0x57, 0x42, 0xf3, 0xfb, 0x22, 0xc0, 0x21, 0x8a, 0xfd, 0x0c, 0xba,
	0x14, 0xbb, 0x93, 0x2e, 0xc5, 0x1b, 0x44, 0x5d, 0x69, 0x77, 0xd1, 0x17, 0x82, 0xff, 0xcf, 0x26,
	0x6c, 0xe7, 0x76, 0xbd, 0xfd, 0x92, 0x15, 0x65, 0xbd, 0x52, 0x01, 0x0a, 0x47, 0xb1, 0xb7, 0xb8,
	0x48, 0x5e, 0x78, 0xd5, 0x70, 0xc9, 0xf4, 0x5a, 0x72, 0xbb, 0xcb, 0x9c, 0x2e, 0xc5, 0x67, 0xa3,
	0xad, 0x53, 0xaf, 0x25, 0x31, 0xcc, 0x96, 0x0f, 0x76, 0x6b, 0x22, 0xbd, 0x05, 0x24, 0xfb, 0x09,
	0x9d, 0x55, 0xbb, 0x92, 0xdc, 0x80, 0xff, 0x24, 0xc5, 0xa6, 0xb5, 0x18, 0xc1, 0xc8, 0x88, 0x4f,
	0x93, 0x9f, 0x58, 0x26, 0xe1, 0xed, 0x1f, 0xf5, 0x9b, 0x81, 0x76, 0xcf, 0x3a, 0x3e, 0xb0, 0x46,
	0x14, 0xc8, 0xdf, 0x1f, 0x24, 0x21, 0x78, 0x83, 0xa9, 0x3d, 0x8e, 0xa7, 0x0f, 0x23, 0x32, 0x07,
	0x7c, 0xf4, 0x3a, 0xb8, 0x2c, 0xa2, 0xf7, 0x01, 0xcd, 0x31, 0x2b, 0xe7, 0xb7, 0x95, 0x7b, 0xb7,
	0xae, 0xca, 0xa2, 0xd4, 0xad, 0x31, 0x74, 0xa8, 0xfb, 0x61, 0x7b, 0x12, 0xc0, 0x34, 0x63, 0x5a,
	0x73, 0xd9, 0x1a, 0x52, 0x87, 0x4e, 0xf3, 0xbe, 0xcd, 0xc6, 0xaf, 0xb8, 0xc9, 0xf3, 0x89, 0x79,
	0xd7, 0xab, 0xef, 0x00, 0x00, 0x00, 0xff, 0xff, 0xf5, 0x2b, 0xaf, 0x9c, 0xf5, 0x01, 0x00, 0x00,
}
