# Armorial-SSL-Robot-STM
 Código de Controle do SSL. Este artigo será secionado em:
 - Informações gerais do STM
 - Comunicação
 - Locomoção
 - Drible
 - Chute

## Informações gerais
 TBD

## Comunicação
 ### Pinagem:
  SDA     | SCL   
  :------:|:-----:
  PB7     | PB6
 
 ### Configuração
  TBD

## Locomoção
 ### Pinagem:
  Roda     | PWM   | Enable   | FWD/REV
  :-------:|:-----:|:--------:|:--------:
  Roda 1   | PB0   | PA4      | PC2
  Roda 2   | PA6   | PC0      | PE6
  Roda 3   | PC7   | PD4      | PA11
  Roda 4   | PC9   | PB3      | PD6
 
 ### Configuração
  - FWD/REV: Todos os pinos configurados como GPIO Output
  - Enable: Todos os pinos configurados como GPIO Output
  - PWM: Todos os pinos configurados nos quatro canais do Timer 3
  - Timer 3: Todos os canais configurados para geração de PWM. A fonte de clock definida deve ser Internal Clock.

  ### Mapeamento das rodas
  Controle | Placa
  --------:|:-----
  Roda 1   | Roda 1   
  Roda 2   | Roda 2   
  Roda 3   | Roda 4   
  Roda 4   | Roda 3
  
## Drible
 TBD

## Chute
 TBD
