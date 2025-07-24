package com.example.demo.service;

import java.time.LocalDateTime;

import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;

import com.example.demo.dto.InputDTO;
import com.example.demo.dto.PointDTO;
import com.example.demo.repository.BoardRepository;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class BoardService {
	private final BoardRepository boardRepository;
	private InputDTO now = null;

	// 로그 작성
	public void writeLog(InputDTO inputDTO) {
		inputDTO.setTime(LocalDateTime.now());
		now = inputDTO;
		boardRepository.writeLog(inputDTO);
	}

	// 가상 포인트 찾기
	public PointDTO findVPoint(int id) {
		return boardRepository.findVPoint(id);
	}

	// 현실 포인트 찾기
	public PointDTO findRPoint(int id) {
		return boardRepository.findRPoint(id);
	}

	// 최신 InputDTO 반환
	public InputDTO getLog() {
		if (now != null)
			return now;
		else
			return null;
	}

}
