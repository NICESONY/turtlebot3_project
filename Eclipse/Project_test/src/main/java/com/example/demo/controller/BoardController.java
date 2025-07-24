package com.example.demo.controller;

import java.util.HashMap;
import java.util.Map;

import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.ResponseBody;

import com.example.demo.dto.*;
import com.example.demo.service.BoardService;
import com.example.demo.websocket.MyWebSocketHandler;


@Controller
@RequiredArgsConstructor
public class BoardController {
	private final BoardService boardService;
	private final MyWebSocketHandler myWebSocketHandler;
	
	@PostMapping("/v_send")
	@ResponseBody
	public String sendVMsg(@RequestBody Integer id) {
		PointDTO pointDTO = boardService.findVPoint(id);
		OutputDTO outputDTO = new OutputDTO();
		outputDTO.setId(0);
		if(id == 0) {
			outputDTO.setX(pointDTO.getX());
			outputDTO.setY(pointDTO.getY());
			outputDTO.setYaw_deg(pointDTO.getYaw_deg());
			outputDTO.setMod(2);
		} else if(id == 9) {
			InputDTO i = boardService.getLog();
			outputDTO.setX(i.getX());
			outputDTO.setY(i.getY());
			outputDTO.setYaw_deg(i.getYaw_deg());
			outputDTO.setMod(3);
		} else if(id == 10) {
			InputDTO i = boardService.getLog();
			outputDTO.setX(i.getX());
			outputDTO.setY(i.getY());
			outputDTO.setYaw_deg(i.getYaw_deg());
			outputDTO.setMod(4);
		} else {
			outputDTO.setX(pointDTO.getX());
			outputDTO.setY(pointDTO.getY());
			outputDTO.setYaw_deg(pointDTO.getYaw_deg());
			outputDTO.setMod(1);
		}
		Map<String, Object> data = new HashMap<>();
		data.put("id", outputDTO.getId());
		data.put("x", outputDTO.getX());
		data.put("y", outputDTO.getY());
		data.put("yaw_deg", outputDTO.getYaw_deg());
		data.put("mod", outputDTO.getMod());
		if(myWebSocketHandler.isSession()) {
			myWebSocketHandler.sendMsg(data);
			return "전송 완료";
		} else {
			return "전송 실패";
		}
			
	}
	
	@PostMapping("/r_send")
	@ResponseBody
	public String sendRMsg(@RequestBody Integer id) {
		PointDTO pointDTO = boardService.findRPoint(id);
		OutputDTO outputDTO = new OutputDTO();
		outputDTO.setId(0);
		if(id == 0) {
			outputDTO.setX(pointDTO.getX());
			outputDTO.setY(pointDTO.getY());
			outputDTO.setYaw_deg(pointDTO.getYaw_deg());
			outputDTO.setMod(2);
		} else if(id == 9) {
			InputDTO i = boardService.getLog();
			outputDTO.setX(i.getX());
			outputDTO.setY(i.getY());
			outputDTO.setYaw_deg(i.getYaw_deg());
			outputDTO.setMod(3);
		} else if(id == 10) {
			InputDTO i = boardService.getLog();
			outputDTO.setX(i.getX());
			outputDTO.setY(i.getY());
			outputDTO.setYaw_deg(i.getYaw_deg());
			outputDTO.setMod(4);
		} else {
			outputDTO.setX(pointDTO.getX());
			outputDTO.setY(pointDTO.getY());
			outputDTO.setYaw_deg(pointDTO.getYaw_deg());
			outputDTO.setMod(1);
		}
		Map<String, Object> data = new HashMap<>();
		data.put("id", outputDTO.getId());
		data.put("x", outputDTO.getX());
		data.put("y", outputDTO.getY());
		data.put("yaw_deg", outputDTO.getYaw_deg());
		data.put("mod", outputDTO.getMod());
		if(myWebSocketHandler.isSession()) {
			myWebSocketHandler.sendMsg(data);
			return "전송 완료";
		} else {
			return "전송 실패";
		}
	}
	
	@GetMapping("/getLog")
	@ResponseBody
	public ResponseEntity<InputDTO> getLog(){
		return ResponseEntity.ok(boardService.getLog());
	}
	
	@GetMapping("/v_turtlebot")
	public String vMove() {
		return "v_turtlebot";
	}
	
	@GetMapping("/r_turtlebot")
	public String rMove() {
		return "r_turtlebot";
	}
}
