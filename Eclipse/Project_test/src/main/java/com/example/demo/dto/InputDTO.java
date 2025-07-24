package com.example.demo.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;
import java.time.LocalDateTime;

@Getter
@Setter
@ToString
public class InputDTO {
	private LocalDateTime time;
	private int id;
	private float x;
	private float y;
	private float yaw_deg;
	private String mod;
	private int batt;
}