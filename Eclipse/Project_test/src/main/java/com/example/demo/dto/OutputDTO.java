package com.example.demo.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class OutputDTO {
	private int id;
	private float x;
	private float y;
	private float yaw_deg;
	private int mod;
}