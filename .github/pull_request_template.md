## Що змінено
<!-- 1-3 речення про суть зміни і навіщо. -->

## Як перевірити
<!-- Команди QA / тести / приклад вхідних даних. -->

## Класифікація PR
- [ ] функціональна зміна (поведінка коду)
- [ ] non-functional: refactor / docs / build / CI
- [ ] зміна протоколу або публічного API (потребує bumped version)

## MilTech checklist
- [ ] **Memory safety**: null-check перед dereference, без off-by-one, RAII замість raw `new`/`delete`
- [ ] **Error paths**: визначена поведінка при failed file open / sensor timeout / value out-of-range
- [ ] **Hardcoded values**: координати / частоти / ліміти / порти - у config або named `constexpr`, не у `.cpp`
- [ ] **Out-of-bounds**: `.at()` де треба, без `strcpy`, перевірка `sizeof` перед `memcpy`
- [ ] **Secrets**: нема API-ключів / паролів / `.env` / `*.key` / `*.pem` у diff
- [ ] **Logging**: нема sensitive data (координати цілей, mission ID, токени, PII) у логах

<!-- Деталі і посилання на MISRA / CERT / CWE - у repository/preps/pr_template_miltech.md -->

## Build / tests
- [ ] `cmake --build` без warnings; CI зелений
- [ ] якщо є тести - запущені; для нової фічі / fix додано регрешн тести
