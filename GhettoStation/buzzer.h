
// 2015 by CopterFail

//ToDo remove tone....

enum eBuzzerStatus { BUZZER_IDLE=0, BUZZER_WARN, BUZZER_ALARM };

class cBuzzer{
	eBuzzerStatus ui8Status;

public:
	cBuzzer(){
		ui8Status = BUZZER_IDLE;
	};

	void vsetStatus( eBuzzerStatus ui8NewStatus ){
		ui8Status = ui8NewStatus;
		playTones();
	};

	void vUpdate( void ){
		playTones();
	}

	eBuzzerStatus getStatus( void ){
		return ui8Status;
	};

	void vClick( void ){
		if( ui8Status == BUZZER_IDLE )
		{
			tone(BUZZER_PIN,1600,40);
		}
	}

private:

	void playTones( void ) {
	    static int toneCounter = 0;
	    toneCounter += 1;
	    switch  (toneCounter) {
	        case 1:
	            tone(BUZZER_PIN ,  1047, 100); break;
	        case 4:
	            if (ui8Status == BUZZER_WARN )
	                tone(BUZZER_PIN , 1047,100);
	            else if (ui8Status == BUZZER_ALARM )
	                tone(BUZZER_PIN , 1047,500);
	            break;
	        case 50:
	            if (ui8Status == BUZZER_ALARM) {
	                toneCounter = 0;
	            }
	            break;
	        case 100:
	            if (ui8Status == BUZZER_WARN) {
	                toneCounter = 0;
	            }
	            break;
	        default:
	            break;
	    }
	}

};
