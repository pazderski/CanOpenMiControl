#pragma once

struct RecordData0 {
    double data;
};

struct RecordData
{
    double data1;
    double data2;
};

struct RecordData2
{
    double data1;
    double data2;
};

struct RecordData4Floats
{
    float data1;
    float data2;
    float data3;
    float data4;
};

struct RecordData6Floats
{
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
};

struct RecordData8Floats
{
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
    float data8;
};

struct RecordData4Uint324Floats
{
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
    uint32_t data4;
    float data5;
    float data6;
    float data7;
    float data8;
};


struct RecordData4int32
{
    int32_t data1;
    int32_t data2;
    int32_t data3;
    int32_t data4;
};

struct RecordData4Uint32
{
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
    uint32_t data4;
};


struct RecordData8Uint32
{
    int32_t data1;
    int32_t data2;
    int32_t data3;
    int32_t data4;
    int32_t data5;
    int32_t data6;
    int32_t data7;
    int32_t data8;
};

struct RecordData5Floats
{
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
};

struct RecordData7Floats
{
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
};

struct RecordData2Floats
{
    float data1;
    float data2;
};

struct RecordData3Floats
{
    float data1;
    float data2;
    float data3;
};

struct RecordData2Doubles
{
    double data1;
   double data2;
};

struct RecordData3Doubles
{
    double data1;
    double data2;
    double data3;
};

struct RecordData4Doubles
{
    double data1;
    double data2;
    double data3;
    double data4;
};

struct RecordData4
{
    double data1;
    double data2;
    double data3;
    double data4;
};

struct RecordData6Doubles
{
   double data1;
   double data2;
   double data3;
   double data4;
   double data5;
   double data6;
};


template < uint32_t size, uint32_t divider, class T >
class DataRecorder
{
	volatile uint32_t sampleNumber;
	uint32_t counter;
public:
	uint32_t delay;
	volatile bool isRecorded;
	T * data;

	static const uint32_t buf_size = size;

	DataRecorder(void * data)
	{
		this->data = (T*) data;
	    sampleNumber = 0;
		counter = 0;
		isRecorded = false;
	}


	void Start()
	{
		sampleNumber = 0;
		isRecorded = false;
	}

	bool Record(T d)
	{
	    counter++;
	    if (counter == divider)
	    {
	        counter = 0;
	        if (sampleNumber == size)
	        {
	            isRecorded = true;
 	            return false;
	        }
	        data[sampleNumber++] = d;
	        return true;
	    }
	    return true;
	}

	void RecordCyclically(T d)
	{
	    counter++;
	    if (counter == divider)
	    {
	        counter = 0;
	        data[sampleNumber++] = d;
	        if (sampleNumber == size)
	            sampleNumber = 0;
	    }

	}

};
