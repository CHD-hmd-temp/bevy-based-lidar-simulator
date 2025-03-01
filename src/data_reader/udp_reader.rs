#![allow(dead_code)]
use std::net::UdpSocket;
use byteorder::{LittleEndian, ReadBytesExt};
use std::time::{Duration, Instant};
use std::io::{Cursor, Error, ErrorKind};
use crate::data_reader::structor::LaserPoint;

fn parse_udp_packet(data: &[u8]) -> Result<Vec<LaserPoint>, Error> {
    const HEADER_SIZE: usize = 36;
    const POINT_SIZE: usize = 14;

    if data.len() < HEADER_SIZE {
        return Err(Error::new(
            ErrorKind::UnexpectedEof,
            format!("Packet too small ({} < {})", data.len(), HEADER_SIZE),
        ));
    }

    let mut cursor = Cursor::new(data);

    // 解析数据包头部(LaserPoint&IMU数据包)
    let _version = cursor.read_u8()?; // 0: 协议版本
    let length = cursor.read_u16::<LittleEndian>()?; // 1: UDP 包长度
    let _time_interval = cursor.read_u16::<LittleEndian>()?; // 3-4: 时间间隔
    let dot_num = cursor.read_u16::<LittleEndian>()?; // 5-6: data包含点云数量
    let _udp_cnt = cursor.read_u16::<LittleEndian>()?; // 7-8: UDP包计数
    let _farme_cnt = cursor.read_u8()?; // 9: 帧计数
    let data_type = cursor.read_u8()?; // 10: 数据类型
    let _time_type = cursor.read_u8()?; // 11: 时间戳类型
    let _reserved1 = cursor.read_u8()?; // 12: 保留字段
    let _crc32 = cursor.read_u32::<LittleEndian>()?; // 13-16: CRC32校验码
    let _timestamp = cursor.read_u64::<LittleEndian>()?; // 17-24: 时间戳

    if data_type != 0x01 {
        return Err(Error::new(
            ErrorKind::InvalidData,
            format!("Unsupported data type: {}", data_type),
        ));
    }

    let length_usize = length as usize;
    if length_usize > data.len() {
        return Err(Error::new(
            ErrorKind::InvalidData,
            format!("Length field ({}) exceeds actual data ({})", length, data.len()),
        ));
    }

    // 解析点云数据
    let payload = &data[HEADER_SIZE..length_usize];
    if payload.len() % POINT_SIZE != 0 || dot_num as usize != payload.len() / POINT_SIZE {
        return Err(Error::new(
            ErrorKind::InvalidData,
            format!("Payload size {} is not multiple of point size {}", payload.len(), POINT_SIZE),
        ));
    }

    let point_count = payload.len() / POINT_SIZE;
    let mut points = Vec::with_capacity(point_count);
    let mut payload_cursor = Cursor::new(payload);

    for _ in 0..point_count {
        let x = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let y = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let z = payload_cursor.read_i32::<LittleEndian>()? as f32 / 1000.0;
        let reflectivity = payload_cursor.read_u8()?;
        let _tag = payload_cursor.read_u8()?;

        if x == 0.0 && y == 0.0 && z == 0.0 {
            continue;
        }

        points.push(LaserPoint {
            x,
            y,
            z,
            reflectivity,
            //tag,
        });
    }

    Ok(points)
}

pub fn read_udp_packets(socket: &UdpSocket, duration: u64) -> std::io::Result<Vec<LaserPoint>> {
    // let socket = UdpSocket::bind("0.0.0.0:56301")?;
    // println!("Listening for UDP data on port 56301..");

    let mut buf = [0; 65536];
    let mut data_buffer = Vec::new();
    let mut data_string = Vec::new();
    let start_time = Instant::now();

    loop {
        match socket.recv_from(&mut buf) {
            Ok((size, _addr)) => {
                data_buffer.extend_from_slice(&buf[..size]);

                match parse_udp_packet(&data_buffer) {
                    Ok(points) => {
                        //println!("Received {} points", points.len());
                        data_string.extend(points);
                    }
                    Err(e) => {
                        eprintln!("Error parsing UDP packet: {}", e);
                    }
                }

                data_buffer.clear();

                if start_time.elapsed() > Duration::from_millis(duration) {
                    // let data_to_output = data_string.clone();
                    // data_string.clear();
                    return Ok(data_string);
                }
            }
            Err(e) => {
                eprintln!("Error receiving UDP packet: {}", e);
                continue;
            }
            
        }
    }
}