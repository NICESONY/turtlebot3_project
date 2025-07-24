package com.example.demo.websocket;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.JsonNode;
import com.example.demo.dto.InputDTO;
import com.example.demo.service.BoardService;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@Component
@Slf4j
@RequiredArgsConstructor
public class MyWebSocketHandler extends TextWebSocketHandler {

    private final ObjectMapper objectMapper = new ObjectMapper();

    private WebSocketSession clientSession;
    private final BoardService boardService;

    @Override
    public void afterConnectionEstablished(WebSocketSession session) throws Exception {
        log.info("WebSocket connection established with session: {}", session.getId());
        this.clientSession = session;
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        String receivedPayload = message.getPayload();
        log.info("Received message from client {}: {}", session.getId(), receivedPayload);

        try {
            JsonNode jsonNode = objectMapper.readTree(receivedPayload);
            InputDTO inputDTO = new InputDTO();
            int mod;
            if (jsonNode.has("id") && jsonNode.get("id").canConvertToInt()) {
                inputDTO.setId(jsonNode.get("id").asInt());
            } else {
                inputDTO.setId(0);
            }
            if (jsonNode.has("x") && jsonNode.get("x").isFloatingPointNumber()) {
                inputDTO.setX((float)jsonNode.get("x").asDouble());
            } else {
                inputDTO.setX(0.0f);
            }
            if (jsonNode.has("y") && jsonNode.get("y").isFloatingPointNumber()) {
                inputDTO.setY((float)jsonNode.get("y").asDouble());
            } else {
                inputDTO.setY(0.0f);
            }
            if (jsonNode.has("yaw_deg") && jsonNode.get("yaw_deg").isFloatingPointNumber()) {
                inputDTO.setYaw_deg((float)jsonNode.get("yaw_deg").asDouble());
            } else {
                inputDTO.setYaw_deg(0.0f);
            }
            if (jsonNode.has("mod") && jsonNode.get("mod").canConvertToInt()) {
                mod = jsonNode.get("mod").asInt();
            } else {
                mod = 0;
            }
            if (jsonNode.has("batt") && jsonNode.get("batt").canConvertToInt()) {
                inputDTO.setBatt(jsonNode.get("batt").asInt());
            } else {
                inputDTO.setBatt(0);
            }
            
            switch (mod) {
			case 0: {
				inputDTO.setMod("Off");
				break;
			}
			case 1: {
				inputDTO.setMod("Activity");
				break;
			}
			case 2: {
				inputDTO.setMod("Charge");
				break;
			}
			case 3: {
				inputDTO.setMod("Stop");
				break;
			}
			default: {
				inputDTO.setMod("Error");
			}}
            boardService.writeLog(inputDTO);
            
        } catch (IOException e) {
            log.error("Error parsing JSON message from client: {}", e.getMessage());
            Map<String, String> errorMessage = new HashMap<>();
            errorMessage.put("type", "error");
            errorMessage.put("message", "Invalid JSON format received.");
            session.sendMessage(new TextMessage(objectMapper.writeValueAsString(errorMessage)));
        }
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) throws Exception {
        log.info("WebSocket connection closed with session {}. Status: {}", session.getId(), status);
        if (this.clientSession != null && this.clientSession.getId().equals(session.getId())) {
            this.clientSession = null;
        }
    }

    @Override
    public void handleTransportError(WebSocketSession session, Throwable exception) throws Exception {
        log.error("WebSocket transport error for session {}: {}", session.getId(), exception.getMessage(), exception);
        if (this.clientSession != null && this.clientSession.getId().equals(session.getId())) {
            this.clientSession = null;
        }
    }
    
    public void sendMsg(Map<String, Object> data) {
        if (this.clientSession != null && this.clientSession.isOpen()) {
            try {
                String jsonMessage = objectMapper.writeValueAsString(data);
                log.info("Sending ROSDTO message to client session {}: {}", clientSession.getId(), jsonMessage);
                this.clientSession.sendMessage(new TextMessage(jsonMessage));
            } catch (IOException e) {
                log.error("Error sending message to client session {}: {}", clientSession.getId(), e.getMessage());
            }
        } else {
            log.warn("Cannot send ROSDTO message: No client session is connected or session is closed.");
        }
    }
    
    public boolean isSession() {
    	if(this.clientSession != null && this.clientSession.isOpen())
    		return true;
    	else
    		return false;
    }
}
