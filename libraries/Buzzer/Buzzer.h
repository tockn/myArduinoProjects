#pragma once

#include <Arduino.h>
#include "TaskBase.h"

#define BUZZER_TASK_PRIORITY    1
#define BUZZER_TASK_STACK_SIZE  2048
#define BUZZER_QUEUE_SIZE       5

class Buzzer : TaskBase {
  public:
    Buzzer(int pin, uint8_t ledc_channel): TaskBase("Buzzer Task", BUZZER_TASK_PRIORITY, BUZZER_TASK_STACK_SIZE), pin(pin), ledc_channel(ledc_channel) {
      playList = xQueueCreate(BUZZER_QUEUE_SIZE, sizeof(enum Music));
      ledcSetup(ledc_channel, 880, 4);
      ledcAttachPin(pin, ledc_channel);
      create_task();
    }
    virtual ~Buzzer() {}
    enum Music {
      BOOT,
      SELECT,
      CONFIRM,
      CANCEL,
      COMPLETE,
      ERROR,
	  SAFE1,SAFE2,SAFE3,SAFE4,WAR1,WAR2,WAR3,
    };
    void play(const enum Music music) {
      xQueueSendToBack(playList, &music, 0);
    }
  private:
    int pin;
    uint8_t ledc_channel;
    xQueueHandle playList;
    void sound(const note_t note, uint8_t octave, uint32_t time_ms) {
      ledcWriteNote(ledc_channel, note, octave);
      vTaskDelay(time_ms / portTICK_RATE_MS);
    }
    void mute(uint32_t time_ms = 400) {
      ledcWrite(ledc_channel, 0);
      vTaskDelay(time_ms / portTICK_RATE_MS);
    }
    void task() {
      while (1) {
        Music music;
        xQueueReceive(playList, &music, portMAX_DELAY);
        switch (music) {
          case BOOT:
            sound(NOTE_B, 5, 200);
            sound(NOTE_E, 6, 400);
            sound(NOTE_Fs, 6, 200);
            sound(NOTE_B, 6, 600);
            mute();
            break;
          case SELECT:
            sound(NOTE_C, 6, 100);
            mute(100);
            break;
          case CONFIRM:
            sound(NOTE_C, 6, 100);
            sound(NOTE_E, 6, 100);
            mute(100);
            break;
          case CANCEL:
            sound(NOTE_E, 6, 100);
            sound(NOTE_C, 6, 100);
            mute(100);
            break;
          case COMPLETE:
            sound(NOTE_C, 6, 100);
            sound(NOTE_D, 6, 100);
            sound(NOTE_E, 6, 100);
            sound(NOTE_F, 6, 100);
            sound(NOTE_G, 6, 100);
            sound(NOTE_A, 6, 100);
            sound(NOTE_B, 6, 100);
            sound(NOTE_C, 7, 100);
            mute(100);
            break;
          case ERROR:
            for (int i = 0; i < 6; i++) {
              sound(NOTE_C, 7, 100);
              sound(NOTE_E, 7, 100);
            }
            mute();
            break;
		  case SAFE1:
			sound(NOTE_C, 6, 100);
			break;
		  case SAFE2:
			sound(NOTE_D, 6, 100);
			break;
		  case SAFE3:
			sound(NOTE_E, 6, 100);
			break;
		  case SAFE4:
			sound(NOTE_F, 6, 100);
			break;
		  case WAR1:
			sound(NOTE_G, 6, 100);
			break;
		  case WAR2:
			sound(NOTE_A, 6, 100);
			break;
		  case WAR3:
			sound(NOTE_B, 6, 100);
			break;

          default:
            sound(NOTE_C, 4, 1000);
            mute();
            break;
        }
      }
    }
};