template<typename T>
class _MultipodController<T, 6> {
protected:
    _BodyModel<T, 6> * body;
    _Locomotion<T> * locomotion;

public:
	_MultipodController() {}

    void set_body(_BodyModel<T, 6> * body) {
        this->body = body;
    }
    void set_locomotion(_Locomotion<T> * locomotion) {
        this->locomotion = locomotion;
    }

    virtual void begin() {}
    virtual void end() {}
    
    virtual void loop(timestamp_t now, timestamp_t last_now) {
        bool support[6] = {true,true,true,true,true,true};
		//print_arr<T, 12>(body->current_joints, "Multipod Drive - Current joints");

        Serial.print(body->transform.val(0, 3)); Serial.print(' ');
        Serial.print(body->transform.val(1, 3)); Serial.print(' ');
        Serial.print(body->transform.val(2, 3)); Serial.print(' ');
        Serial.println();

        this->body->inverse(support, 0.1, 10);
    }
};
