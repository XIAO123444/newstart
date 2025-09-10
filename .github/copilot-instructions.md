# Copilot Instructions for smartcarcode

## ��Ŀ�ܹ�����ҪĿ¼

- `CarCode/`��������Ŀ¼���������Ĵ��루`code/`����IAR/MDK �����ļ������Ŀ¼�ȡ�
  - `code/`����Ҫ C ���룬��Ϊ������ģ�飨�� `buzzer.c/h`, `encoder.c/h`, `menu.c/h`, `screen.c/h` �ȣ���
  - `iar/`, `mdk/`���ֱ�Ϊ IAR �� Keil MDK �������ü���ؽű���
- `libraries/`���ⲿ������ͨ�ÿ⣬���� `zf_common/`��ͨ�ù��ߣ���`zf_device/`���豸��������`sdk/`��оƬ��أ���`components/`���� fatfs �ļ�ϵͳ����

## �ؼ���������

- **����/��¼/����**��
  - �Ƽ�ʹ�� IAR �� Keil MDK �����ļ����п�����
  - VSCode �¿��� CMSIS ��� task���硰CMSIS Load������CMSIS Run������� pyocd ���߽�����¼�͵��ԡ�
- **��������**��
  - ��¼��`pyocd load --probe cmsisdap: --cbuild-run ...`
  - ������`pyocd erase --probe cmsisdap: --chip ...`
  - ���ԣ�`pyocd gdbserver --probe cmsisdap: --connect attach ...`

## ��ĿԼ������

- ͷ�ļ����ò������·�������ù���ͷ�ļ��� `zf_common_headfile.h`��
- ���Ͷ�������ʹ�� `<stdint.h>`���� `int16_t`���������Զ����������� `int16`��
- �����Ϊ����ģ�飬ÿ��ģ���ж����� `.c`/`.h` �ļ����ӿ�ͨ��ͷ�ļ���¶��
- �豸/������ش��뼯���� `zf_device/`��ͨ�ù����� `zf_common/`��

## �����뼯��

- ����оƬ SDK��`sdk/`����������������� `fatfs`����
- ͨ�� pyocd ֧�� CMSIS-DAP ��������
- ���������ļ����� `.csolution.yml`��`.cbuild.yml`��λ�� `mdk/` Ŀ¼��

## ����ģʽʾ��

- ͷ�ļ�������
  ```c
  #ifndef CODE_SCREEN_H_
  #define CODE_SCREEN_H_
  // ...
  #endif
  ```
- ���ܽӿڱ�¶��
  ```c
  void show_line(void);
  ```
- ͷ�ļ����ã�
  ```c
  #include "zf_common_headfile.h"
  #include <stdint.h>
  ```

## ����˵��

- ������ͷ�ļ��Ҳ��������ȼ�� `libraries/` �����Ŀ¼�� include ·����
- ������ƫ��Ƕ��ʽ C��ע��ģ�黯��Ӳ������

---
���貹����幤������������Լ�������ڴ��ĵ�����˵����
