package com.example.demo.repository;

import org.mybatis.spring.SqlSessionTemplate;
import org.springframework.stereotype.Repository;

import com.example.demo.dto.InputDTO;
import com.example.demo.dto.PointDTO;

import lombok.RequiredArgsConstructor;

@Repository
@RequiredArgsConstructor
public class BoardRepository {
	private final SqlSessionTemplate sql;

	// 로그 작성
	public void writeLog(InputDTO inputDTO) {
		sql.insert("Board.writeLog", inputDTO);
	}

	// 가상 포인트 찾기
	public PointDTO findVPoint(int id) {
		return sql.selectOne("Board.findVPoint", id);
	}

	// 현실 포인트 찾기
	public PointDTO findRPoint(int id) {
		return sql.selectOne("Board.findRPoint", id);
	}
}
