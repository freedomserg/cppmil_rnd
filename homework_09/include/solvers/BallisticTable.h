#pragma once

#include <string>
#include <vector>

// 5-вимірна сітка заздалегідь обчислених результатів балістики
// з нерівномірним кроком по кожній осі. Між вузлами — багатовимірна
// лінійна інтерполяція, за межами — крайнє значення (clamp).
struct BallisticTable {
    // Осі — кожна зі своїм набором відсортованих вузлів
    std::vector<float> axisZ0;  // висота
    std::vector<float> axisV0;  // швидкість
    std::vector<float> axisM;   // маса
    std::vector<float> axisD;   // опір
    std::vector<float> axisL;   // підйомна сила

    // Результат у кожному вузлі сітки
    struct Result {
        float t;      // час польоту
        float hDist;  // горизонтальна дистанція
    };

    // Плоский масив розміром |Z0| * |V0| * |M| * |D| * |L|
    std::vector<Result> data;

    // Індекс у плоскому масиві для вузла [iz][iv][im][id][il]
    size_t index(int iz, int iv, int im, int id, int il) const;
    const Result& at(int iz, int iv, int im, int id, int il) const;

    // Завантаження з текстового файлу. 
    bool load(const std::string& path);

    // Багатовимірна лінійна інтерполяція по 5 осях (з clamp за межами таблиці).
    Result lookup(float Z0, float V0, float m, float d, float l) const;
};
