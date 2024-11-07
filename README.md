# Motion Profile Library

Una libreria Python per generare profili di movimento lineari, utilizzando parametri personalizzabili come distanza, accelerazione, velocità e tempo totale. Questa libreria è ideale per la creazione e gestione di profili di velocità in applicazioni di automazione industriale, robotica e controllo del movimento.

## Funzionalità

Questa libreria permette di:

- Generare un profilo di velocità lineare che include le fasi di accelerazione, velocità costante e decelerazione.
- Gestire i parametri di accelerazione minima e massima, velocità minima e massima, e tempo totale.
- Personalizzare la percentuale del tempo di movimento in cui la velocità è costante.
- Adattare i parametri per garantire che il profilo soddisfi i vincoli imposti su tempo e distanza.

###Parameters
Parametri
- distance: Distanza totale del movimento in mm.
- time_total: Tempo totale per il movimento in secondi.
- acc_min: Accelerazione minima in mm/s².
- acc_max: Accelerazione massima in mm/s².
- vel_min: Velocità minima in mm/s.
- vel_max: Velocità massima in mm/s.
- percentage_constant_speed: Percentuale del tempo totale trascorso a velocità costante (default: 0.25).
#### Metodi Principali
- calculate_time_phases(): Calcola i tempi per le fasi di accelerazione, velocità costante e decelerazione.
- generate_velocity_profile(): Genera il profilo di velocità basato sui parametri impostati.
- adjust_parameters(): Aggiusta velocità e accelerazione per soddisfare i vincoli di tempo e distanza.
####Contributi
Se desideri contribuire al progetto, sentiti libero di fare un fork del repository e inviare una pull request con le tue modifiche. Siamo sempre alla ricerca di miglioramenti e nuove funzionalità.

## Installazione

Clona il repository ed entra nella cartella del progetto:

```bash
git clone https://github.com/daddi1987/MotionProfile.git
cd MotionProfile
