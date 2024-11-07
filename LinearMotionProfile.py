import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from matplotlib.widgets import Cursor
from matplotlib.animation import FuncAnimation


class ProfileGenerator:

    found = False
    best_vel = None
    best_acc = None
    best_t_acc = None
    best_t_dec = None
    best_t_const = None

    def __init__(self,M1_Acc_Dec, M1_MaxSpeed, M2_Acc_Dec, M2_MaxSpeed,mm_per_revolution,FactorGroup, percentage_constant_speed=0.25, modetype="XY Core Frame", acc_min=0.5, vel_min =0.1,acc_max = 2.0,vel_max = 0.5,StrokeThreshold = 100):

        """
        Inizializza i parametri per la generazione del profilo di movimento.
        """

        #self.Xstart = Xstart
        #self.Ystart = Ystart
        #self.X = X
        #self.Y = Y
        self.M1_Acc_Dec = M1_Acc_Dec
        self.M1_MaxSpeed = M1_MaxSpeed
        self.M2_Acc_Dec = M2_Acc_Dec
        self.M2_MaxSpeed = M2_MaxSpeed
        self.mm_per_revolution = mm_per_revolution
        self.percentage_constant_speed = percentage_constant_speed
        self.modetype = modetype
        self.acc_min = acc_min
        self.vel_min = vel_min
        self.acc_max = acc_max
        self.vel_max = vel_max
        self.StrokeThreshold = StrokeThreshold
        self.FactorGroup = FactorGroup


    def SyncCoreXYAxis(self, Xstart, Ystart, X, Y, Graph=True):

        print("Generate Sync for CORE XY")

        M1_mm, M2_mm = self.calculate_theta(X,Y,Xstart,Ystart)   # Ricalculate real displacement in millimeter

        #Inversione del segno se necesario
        if M1_mm < 0:
            M1_mm = M1_mm * -1
        if M2_mm < 0:
            M2_mm = M2_mm * -1

        #Check Max displacement axis and decide when is the max stroke axis

        if ((X < self.StrokeThreshold) and (Y < self.StrokeThreshold)):
            print("Traiettoria troppo corta utilizzare velocità minime..................")
            (X_Trajectory, Y_Trajectory, M1_TrajectoryTime, M2_TrajectoryTime, M1_AccTime, M2_AccTime, Tj_Stroke_M1,
             Tj_Stroke_M2, M1_TrajectoryTotalTime, M2_TrajectoryTotalTime, M1_TimeVcons,
             M2_TimeVcons) = self.TrajectoryGenerator(M1_mm, M2_mm,self.vel_min,self.vel_min,self.acc_min,self.acc_min,self.mm_per_revolution)

        else:

            # ------------------------GENERATORE DI TRAIETTORIA, GENERA ENTRAMBE LE TRAIETTORIE DEI MOTORI----------------------------------

            (X_Trajectory, Y_Trajectory, M1_TrajectoryTime, M2_TrajectoryTime, M1_AccTime, M2_AccTime, Tj_Stroke_M1,
             Tj_Stroke_M2, M1_TrajectoryTotalTime, M2_TrajectoryTotalTime, M1_TimeVcons,
             M2_TimeVcons) = self.TrajectoryGenerator(M1_mm, M2_mm,self.M1_MaxSpeed,self.M2_MaxSpeed,self.M1_Acc_Dec,self.M1_Acc_Dec,self.mm_per_revolution)



        if Tj_Stroke_M1 > Tj_Stroke_M2:
            print("Y Axis too Slow - Ricalculate this trajectory")
            print("\n")
            AxisStroke = Tj_Stroke_M2  # mm
            TimeTrajectory = M1_TrajectoryTotalTime  # s
            colorAxis = "green"
            colorAxis1 = "blue"
            TimeTraj = M1_TrajectoryTime
            SpeedTraj = X_Trajectory
            PositionXAxis = M1_mm
            PositionYAxis = M2_mm
            TimeYAxis = M2_TrajectoryTotalTime
            AccAxisX = self.M2_Acc_Dec / 1000
            MaxSpeedXAxis = self.M2_MaxSpeed / 1000
            TimeAccTrajectorySyn = M1_AccTime
            TimeDecTrajectorySyn = M1_AccTime
            TimeSpeedTrjectorySyn = M1_TimeVcons
            AccTimeTrajectoryRef = M2_AccTime
            AxisRecalculate = "Y"
            PositionXAxis = M1_mm
            PositionYAxis = M2_mm
            TimeXAxis = M1_TrajectoryTotalTime
            AccAxisX = self.M1_Acc_Dec * 1000
            MaxSpeedXAxis = self.M1_MaxSpeed * 1000
            # TimeYAxis = StrokeTime_Eq
            # AccAxisY = acceleration_Eq
            # MaxSpeedYAxis = max_velocity_Eq
            MaxTimeAxis = TimeXAxis
            TimeAccX = M1_AccTime
            # TimeAccY = TimeAcc_Eq
            # --------------------RICALCOLARE IL PROFILO DI VELOCITA'-------------------------#
            acc_Y_new, max_speed_Y_new = self.calculate_trajectoryAccPhaseSync(TimeAccX, PositionYAxis, TimeXAxis)

            t_acc, t_const, t_dec, total_time, TimeAlghorytmics, SpeedAlghorytmics = self.SingleTrajectoryGenerator(AxisStroke,max_speed_Y_new,acc_Y_new)

            MaxSpeed_Flag = max_speed_Y_new
            AccAxisY = acc_Y_new
            MaxSpeedYAxis = max_speed_Y_new
            TimeYAxis = total_time

            # ------------------------------------FINE-----------------------------------------

        else:
            print("X Axis too Slow - Ricalculate this trajectory")
            print("\n")
            AxisStroke = Tj_Stroke_M1  # mm
            TimeTrajectory = M2_TrajectoryTotalTime  # s
            colorAxis = "blue"
            colorAxis1 = "green"
            TimeTraj = M2_TrajectoryTime
            SpeedTraj = Y_Trajectory
            PositionXAxis = M1_mm
            PositionYAxis = M2_mm
            TimeXAxis = M1_TrajectoryTotalTime
            AccAxisX = self.M1_Acc_Dec / 1000
            MaxSpeedXAxis = self.M1_MaxSpeed / 1000
            TimeAccTrajectorySyn = M2_AccTime
            TimeDecTrajectorySyn = M2_AccTime
            TimeSpeedTrjectorySyn = M2_TimeVcons
            AccTimeTrajectoryRef = M1_AccTime
            AxisRecalculate = "X"
            PositionXAxis = M1_mm
            PositionYAxis = M2_mm
            # TimeXAxis = StrokeTime_Eq
            # AccAxisX = acceleration_Eq
            # MaxSpeedXAxis = max_velocity_Eq
            TimeYAxis = M2_TrajectoryTotalTime
            AccAxisY = self.M2_Acc_Dec * 1000
            MaxSpeedYAxis = self.M2_MaxSpeed * 1000
            MaxTimeAxis = TimeYAxis
            # TimeAccX = TimeAcc_Eq
            TimeAccY = M1_AccTime
            # --------------------RICALCOLARE IL PROFILO DI VELOCITA'-------------------------#
            acc_X_new, max_speed_X_new = self.calculate_trajectoryAccPhaseSync(TimeAccY, PositionXAxis, TimeYAxis)

            t_acc, t_const, t_dec, total_time, TimeAlghorytmics, SpeedAlghorytmics = self.SingleTrajectoryGenerator(AxisStroke,max_speed_X_new,acc_X_new)

            MaxSpeed_Flag = max_speed_X_new
            AccAxisX = acc_X_new
            MaxSpeedXAxis = max_speed_X_new
            TimeXAxis = total_time

        # Supponiamo di avere le posizioni calcolate e i tempi per X e Y
        TimeX, PositionXAxis = self.KinematicPosition(PositionXAxis, TimeXAxis, AccAxisX, MaxSpeedXAxis,
                                                 self.percentage_constant_speed)
        TimeY, PositionYAxis = self.KinematicPosition(PositionYAxis, TimeYAxis, AccAxisY, MaxSpeedYAxis,
                                                 self.percentage_constant_speed)

        # Calcoliamo le differenze delle posizioni (distanze tra punti consecutivi)
        distance_X = np.diff(PositionXAxis)
        distance_Y = np.diff(PositionYAxis)

        # Trova la lunghezza minima tra distance_X e distance_Y
        min_length = min(len(distance_X), len(distance_Y))

        # Taglia gli array alla lunghezza minima
        distance_X = distance_X[:min_length]
        distance_Y = distance_Y[:min_length]

        # Calcola la distanza totale percorsa sul piano XY
        distance_total = np.sqrt(distance_X ** 2 + distance_Y ** 2)

        # Normalizzazione delle distanze per evitare allungamenti eccessivi
        distance_max = min(max(PositionXAxis), max(PositionYAxis))
        distance_total_normalized = np.clip(distance_total, a_min=0, a_max=distance_max)

        # Normalizziamo il tempo in funzione della lunghezza minima di distance_total
        time_total = np.linspace(0, min(max(TimeX), max(TimeY)), len(distance_total_normalized))

        # Interpoliamo le posizioni X e Y su un dominio temporale comune
        interp_X = interp1d(TimeX, PositionXAxis, kind='linear')
        interp_Y = interp1d(TimeY, PositionYAxis, kind='linear')

        # Posizioni interpolate sul tempo comune
        position_M1_interpolated = interp_X(time_total)
        position_M2_interpolated = interp_Y(time_total)

        Rev_MaxSpeedXAxis = (MaxSpeedXAxis * self.mm_per_revolution) / self.FactorGroup
        Rev_AccAxisX = (AccAxisX * self.mm_per_revolution) / self.FactorGroup
        Rev_MaxSpeedYAxis = (MaxSpeedYAxis * self.mm_per_revolution) / self.FactorGroup
        Rev_AccAxisY = (AccAxisY * self.mm_per_revolution) / self.FactorGroup



        if Graph == True:
            # Creazione della figura con due riquadri (subplots)
            fig, axs = plt.subplots(2, 2, figsize=(12, 10))

            # Grafico 1: Profilo di velocità
            axs[0, 0].plot(M1_TrajectoryTime, X_Trajectory, label="Velocità Asse X (mm/s)", color="blue")
            # axs[0,0].scatter(t, velocity_profile, color='orange', s=10)  # Aggiungi i punti campionati
            axs[0, 0].plot(M2_TrajectoryTime, Y_Trajectory, label="Velocità Asse Y (mm/s)", color="green")
            # axs[0,0].scatter(M2_TrajectoryTime, Y_Trajectory, color='yellow', s=10)  # Aggiungi i punti campionati
            axs[0, 0].set_title("Profilo di Velocità")
            axs[0, 0].set_xlabel("Tempo (s)")
            axs[0, 0].set_ylabel("Velocità (mm/s)")
            axs[0, 0].grid(True)
            axs[0, 0].legend()

            # Grafico 2: Profilo di velocità e accelerazione
            axs[0, 1].plot(TimeAlghorytmics, SpeedAlghorytmics, label='Velocità Asse X (mm/s)', color=colorAxis)
            axs[0, 1].plot(TimeTraj, SpeedTraj, label='Velocità Asse Y (mm/s)', color=colorAxis1)
            axs[0, 1].axhline(y=MaxSpeed_Flag, color='red', linestyle='--',
                              label=f'Velocità Massima: {MaxSpeed_Flag:.2f} mm/s')
            # axs[0, 1].axvline(x=acceleration_Eq, color='green', linestyle='--', label='Fine accelerazione')
            # axs[0, 1].axvline(x=best_t_accAlgho + best_time_const, color='orange', linestyle='--', label='Fine velocità costante')
            axs[0, 1].axvline(x=TimeTrajectory, color='purple', linestyle='--', label='Fine movimento')
            axs[0, 1].set_title('Profilo di Velocità e Accelerazione')
            axs[0, 1].set_xlabel('Tempo (s)')
            axs[0, 1].set_ylabel('Velocità (mm/s)')
            axs[0, 1].legend()
            axs[0, 1].grid(True)

            axs3 = axs[1, 0]
            # axs3_1 = axs3.twinx()
            axs3_1 = axs3.twiny()
            # Interpolazione della traiettoria su un piano X-Y
            axs3.plot(TimeX, PositionXAxis, 'b', label="Interpolated Trajectory Axis X")
            axs3.plot(TimeY, PositionYAxis, 'g', label="Interpolated Trajectory Axis Y")
            axs3_1.plot(position_M1_interpolated, position_M2_interpolated, 'r-', label="Traiettoria Interpolata")

            # axs[1, 0].scatter(position_M1_interpolated, position_M2_interpolated, color='blue', marker='o', label='Punti Campionati')  # Grafico a dispersione
            axs[1, 0].set_title("Interpolazione Traiettoria X-Y")
            axs[1, 0].set_xlabel("Posizione Asse X (mm)")
            axs[1, 0].set_ylabel("Posizione Asse Y (mm)")
            # axs[1,0].set_xlim(-460, 460)  # Limite massimo dell'asse X
            # axs[1,0].set_ylim(-600, 600)  # Limite massimo dell'asse Y

            axs[1, 1].axis('off')  # Disattiva il grafico in basso a destra
            fig.text(0.60, 0.05,
                     (f'-                   Dinamiche Utilizzate:\n'
                      f'- \n'
                      f'- Speed Max Axis X (demand): {self.M1_MaxSpeed} mm/s\n'
                      f'- Max Acceleration Axis X(demand): {self.M1_Acc_Dec} mm/s²\n'
                      f'- Speed Max Axis Y(demand): {self.M2_MaxSpeed} mm/s\n'
                      f'- Max Acceleration Axis Y(demand): {self.M2_Acc_Dec} mm/s²\n'
                      f'- Time Trajectory Axis X: {round(M1_TrajectoryTotalTime, 3)} s\n'
                      f'- Time Trajectory Axis Y: {round(M2_TrajectoryTotalTime, 3)} s\n'
                      f'- Corsa Asse X: {X} mm\n'
                      f'- Corsa Asse Y: {Y} mm\n'
                      f'- Asse Ricalcolato:  {AxisRecalculate} \n'
                      f'- \n'
                      f'- Speed Max Axis X: {round(MaxSpeedXAxis, 2)} mm/s\n'
                      f'- Max Acceleration Axis X: {round(AccAxisX, 2)} mm/s²\n'
                      f'- Speed Max Axis Y: {round(MaxSpeedYAxis, 3)} mm/s\n'
                      f'- Max Acceleration Axis Y: {round(AccAxisY, 3)} mm/s²\n'
                      f'- Time Accelerazione Asse X: {round(M1_AccTime, 3)} s\n'
                      f'- Time Accelerazione Asse Y: {round(M2_AccTime, 3)} s\n'
                      f'- \n'
                      f'- Speed Max Axis X : {round(Rev_MaxSpeedXAxis, 2)} (rev/s)\n'
                      f'- Max Acceleration Axis X: {round(Rev_AccAxisX, 2)} (rev/s²)\n'
                      f'- Speed Max Axis Y : {round(Rev_MaxSpeedYAxis, 3)} (rev/s)\n'
                      f'- Max Acceleration Axis Y : {round(Rev_AccAxisY, 3)} (rev/s²)\n'
                      f'- \n'),
                     fontsize=12, color='black', bbox=dict(facecolor='white', alpha=0.5))

            cursor = Cursor(axs[1, 0], useblit=True, color='red', linewidth=1)
            axs[1, 0].legend()
            plt.grid(True)

            # Mostra entrambi i grafici
            plt.tight_layout()
            plt.show()
        return  round(Rev_MaxSpeedXAxis, 2), round(Rev_AccAxisX, 2), round(Rev_MaxSpeedYAxis, 3), round(Rev_AccAxisY, 3), PositionXAxis, PositionYAxis

    def calculate_theta(self, X, Y, Xstart, Ystart):

        delta_X = X - Xstart
        delta_Y = Y - Ystart

        M1 = (-delta_X - delta_Y)
        M2 = (-delta_X + delta_Y)

        M1_LinearStroke = M1
        M2_LinearStroke = M2

        return M1_LinearStroke, M2_LinearStroke

    def TrajectoryGenerator(self, dPosition_M1, dPosition_M2, MaxSpeed_M1, MaxSpeed_M2, AccDec_M1, AccDec_M2,RevolutionMotor):
        aPositionM1 = dPosition_M1 / 100  #Controllare perchè divisione per 100
        aPositionM2 = dPosition_M2 / 100  #Controllare perchè divisione per 100

        dPositionM1 = dPosition_M1
        dPositionM2 = dPosition_M2

        # Conversione da rivoluzioni a mm
        aPositionM1_mm = aPositionM1 * RevolutionMotor
        aPositionM2_mm = aPositionM2 * RevolutionMotor
        Stroke_M1 = dPositionM1
        Stroke_M2 = dPositionM2

        M1_distanceRevolution = Stroke_M1 / RevolutionMotor
        M2_distanceRevolution = Stroke_M2 / RevolutionMotor

        # Velocità massima in mm/s
        MaxSpeed_M1_mm = MaxSpeed_M1 * 1000
        MaxSpeed_M2_mm = MaxSpeed_M2 * 1000
        dSpeedM1_rev = MaxSpeed_M1_mm / RevolutionMotor
        dSpeedM2_rev = MaxSpeed_M2_mm / RevolutionMotor

        # Tempo di percorrenza stimato
        StrokeTime_M1 = Stroke_M1 / MaxSpeed_M1_mm
        StrokeTime_M2 = Stroke_M2 / MaxSpeed_M2_mm

        # Tempo di accelerazione per l'asse M1
        AccDec_M1 = AccDec_M1 * 1000
        M1_AccTime = MaxSpeed_M1_mm / AccDec_M1
        StrokeAccDec_M1 = (MaxSpeed_M1_mm ** 2) / (2 * AccDec_M1)

        # Tempo di accelerazione per l'asse M2
        AccDec_M2 = AccDec_M2 * 1000
        M2_AccTime = MaxSpeed_M2_mm / AccDec_M2
        StrokeAccDec_M2 = (MaxSpeed_M2_mm ** 2) / (2 * AccDec_M2)

        t_const = 0
        t_const_M2 = 0

        # Calcolo del profilo per M1
        if Stroke_M1 < 2 * StrokeAccDec_M1:
            # Profilo triangolare per M1
            StrokeTotalTime_M1 = 2 * (Stroke_M1 / AccDec_M1) ** 0.5
            v_max_reached_M1 = (Stroke_M1 * AccDec_M1) ** 0.5
        else:
            # Profilo trapezoidale per M1
            d_const_M1 = Stroke_M1 - 2 * StrokeAccDec_M1
            t_const = d_const_M1 / MaxSpeed_M1_mm
            StrokeTotalTime_M1 = 2 * M1_AccTime + t_const
            v_max_reached_M1 = MaxSpeed_M1_mm

        # Calcolo del profilo per M2
        if Stroke_M2 < 2 * StrokeAccDec_M2:
            # Profilo triangolare per M2
            StrokeTotalTime_M2 = 2 * (Stroke_M2 / AccDec_M2) ** 0.5
            v_max_reached_M2 = (Stroke_M2 * AccDec_M2) ** 0.5
        else:
            # Profilo trapezoidale per M2
            d_const_M2 = Stroke_M2 - 2 * StrokeAccDec_M2
            t_const_M2 = d_const_M2 / MaxSpeed_M2_mm
            StrokeTotalTime_M2 = 2 * M2_AccTime + t_const_M2
            v_max_reached_M2 = MaxSpeed_M2_mm

        # Generazione del tempo totale per M1 e M2
        t = np.linspace(0, StrokeTotalTime_M1, 1000)
        velocity_profile = np.zeros_like(t)
        t_M2 = np.linspace(0, StrokeTotalTime_M2, 1000)
        velocity_profile_M2 = np.zeros_like(t_M2)

        # Profilo di velocità per M1
        for i, time in enumerate(t):
            if Stroke_M1 < 2 * StrokeAccDec_M1:
                if time <= StrokeTotalTime_M1 / 2:
                    velocity_profile[i] = AccDec_M1 * time
                else:
                    velocity_profile[i] = v_max_reached_M1 - AccDec_M1 * (time - StrokeTotalTime_M1 / 2)
            else:
                if time <= M1_AccTime:
                    velocity_profile[i] = AccDec_M1 * time
                elif time <= M1_AccTime + t_const:
                    velocity_profile[i] = v_max_reached_M1
                else:
                    velocity_profile[i] = v_max_reached_M1 - AccDec_M1 * (time - M1_AccTime - t_const)

        # Profilo di velocità per M2
        for iM2, time_M2 in enumerate(t_M2):
            if Stroke_M2 < 2 * StrokeAccDec_M2:
                if time_M2 <= StrokeTotalTime_M2 / 2:
                    velocity_profile_M2[iM2] = AccDec_M2 * time_M2
                else:
                    velocity_profile_M2[iM2] = v_max_reached_M2 - AccDec_M2 * (time_M2 - StrokeTotalTime_M2 / 2)
            else:
                if time_M2 <= M2_AccTime:
                    velocity_profile_M2[iM2] = AccDec_M2 * time_M2
                elif time_M2 <= M2_AccTime + t_const_M2:
                    velocity_profile_M2[iM2] = v_max_reached_M2
                else:
                    velocity_profile_M2[iM2] = v_max_reached_M2 - AccDec_M2 * (time_M2 - M2_AccTime - t_const_M2)

        print("Tempo Percorrenza Asse X:", StrokeTotalTime_M1)
        print("Tempo Percorrenza Asse Y:", StrokeTotalTime_M2)

        return velocity_profile, velocity_profile_M2, t, t_M2, M1_AccTime, M2_AccTime, Stroke_M1, Stroke_M2, StrokeTotalTime_M1, StrokeTotalTime_M2, t_const, t_const_M2

    def calculate_trajectoryAccPhaseSync(self,time_acc_X_axis, stroke_axis, total_time_trajectory):
        acc_dec = stroke_axis / (time_acc_X_axis * (total_time_trajectory - time_acc_X_axis))
        max_speed = acc_dec * time_acc_X_axis

        # Calcolo del tempo a velocità costante
        time_const = total_time_trajectory - 2 * time_acc_X_axis

        # Se il tempo costante è negativo, ricalcoliamo riducendo la velocità massima
        if time_const < 0:
            # Caso in cui non c'è tempo a velocità costante, dobbiamo ricalcolare la velocità massima
            time_const = 0
            max_speed = stroke_axis / (2 * time_acc_X_axis)  # Nessuna fase a velocità costante

        # Verifica che il tempo totale sia esattamente uguale a total_time_trajectory
        total_time_generated = 2 * time_acc_X_axis + time_const

        if total_time_generated != total_time_trajectory:
            TryReduction = 0
            print("Primo Tentativo di iterazione per arrivare al termine della traiettoria nel tempo prestabilito.")
            while ((total_time_generated > total_time_trajectory) and (TryReduction < 20)):
                max_speed -= 0.01  # Riduciamo leggermente la velocità massima
                acc_dec = max_speed / time_acc_X_axis
                total_time_generated = (2 * time_acc_X_axis) + (stroke_axis / max_speed)
                TryReduction += 1

            if TryReduction == 20:
                print(
                    "iterazioni terminate senza trovare una traiettoria adeguata, sidovrebbe rallentare l'asse principale. Parametri più vicini inseriti")
                acc_dec = self.acc_min
                max_speed = self.vel_min
        else:
            print("Traiettoria asse ricalcolata correttamente")
        return acc_dec, max_speed

    def SingleTrajectoryGenerator(self,dPosition_M1, MaxSpeed_M1, AccDec_M1):

        # Calcolo del tempo necessario per raggiungere la velocità massima
        t_acc = MaxSpeed_M1 / AccDec_M1
        # Calcolo della distanza percorsa durante l'accelerazione e decelerazione (sono simmetriche)
        d_acc = 0.5 * AccDec_M1 * t_acc ** 2

        if 2 * d_acc >= dPosition_M1:
            # Profilo triangolare: la velocità massima non viene mai raggiunta
            t_acc = np.sqrt(dPosition_M1 / AccDec_M1)
            t_dec = t_acc
            t_const = 0
            v_max_actual = AccDec_M1 * t_acc
            d_total = dPosition_M1
        else:
            # Profilo trapezoidale: si raggiunge la velocità massima e si mantiene per un tratto
            d_const = dPosition_M1 - 2 * d_acc
            t_const = d_const / MaxSpeed_M1
            t_dec = t_acc
            v_max_actual = MaxSpeed_M1
            d_total = dPosition_M1

        # Calcolo del tempo totale
        total_time = t_acc + t_const + t_dec

        # Profilo temporale
        time_profile = np.array([0, t_acc, t_acc + t_const, total_time])
        # Profilo velocità
        velocity_profile = np.array([0, v_max_actual, v_max_actual, 0])

        return t_acc, t_const, t_dec, total_time, time_profile, velocity_profile

    def KinematicPosition(self,AxisStroke, TimeTrajectory, acc_max, vel_max, percentuale_velocità_costante):
        # Calcolo del tempo di acce,erazione e decelerazione
        t_acc = vel_max / acc_max  # Tempo di accelerazione e decelerazione
        t_dec = t_acc
        # Determina il tempo di velocità costante come percentuale del tempo totale
        t_const = TimeTrajectory * percentuale_velocità_costante / 100.0

        # Verifica se il profilo di velocità costante è fattibile
        # if t_acc + t_dec > TimeTrajectory:
        # acc_max = acc_min
        # vel_max = vel_min
        # raise ValueError("Il tempo totale di traiettoria non è sufficiente per accelerare e decelerare.")

        # Se la velocità costante è troppo lunga, calcoliamo la velocità massima in base alla corsa
        distance_acc_dec = acc_max * t_acc ** 2  # Distanza totale coperta in accelerazione e decelerazione
        distance_const = AxisStroke - distance_acc_dec  # Distanza da percorrere a velocità costante

        # Ricalcolo del tempo a velocità costante se necessario
        if distance_const < 0:
            # Distanza troppo corta per raggiungere la velocità massima
            t_acc = np.sqrt(AxisStroke / (2 * acc_max))
            t_dec = t_acc
            t_const = 0
            vel_max = acc_max * t_acc
        else:
            t_const = distance_const / vel_max

        # Fase di accelerazione (da 0 a t_acc)
        time_acc = np.linspace(0, t_acc, num=100)
        position_acc = 0.5 * acc_max * time_acc ** 2

        # Fase di velocità costante (da t_acc a t_acc + t_const)
        time_const = np.linspace(t_acc, t_acc + t_const, num=100)
        position_const = position_acc[-1] + vel_max * (time_const - t_acc)

        # Fase di decelerazione (da t_acc + t_const a TimeTrajectory)
        time_dec = np.linspace(t_acc + t_const, TimeTrajectory, num=100)
        position_dec = position_const[-1] + vel_max * (time_dec - (t_acc + t_const)) - 0.5 * acc_max * (
                time_dec - (t_acc + t_const)) ** 2

        # Unisci i risultati di posizione e tempo per ogni fase
        time_profile = np.concatenate([time_acc, time_const, time_dec])
        position_profile = np.concatenate([position_acc, position_const, position_dec])

        return time_profile, position_profile

    def TrajectorySimulator2D (self,X_trajectory,Y_trajectory):
        # Interpolazione delle traiettorie X e Y
        interp_points = 200  # Numero di punti di interpolazione
        t = np.linspace(0, 1, len(X_trajectory))
        t_new = np.linspace(0, 1, interp_points)

        interp_x = interp1d(t, X_trajectory, kind='linear')
        interp_y = interp1d(t, Y_trajectory, kind='linear')

        x_interpolated = interp_x(t_new)
        y_interpolated = interp_y(t_new)

        # Creazione della figura e dell'asse
        fig, ax = plt.subplots()
        ax.set_xlim(0, max(X_trajectory))
        ax.set_ylim(0, max(Y_trajectory))
        ax.set_xlabel("Posizione Asse X (mm)")
        ax.set_ylabel("Posizione Asse Y (mm)")
        ax.set_title("Animazione Traiettoria X-Y")
        ax.plot(x_interpolated, y_interpolated, color='gray', linestyle='-', label="Traiettoria Interpolata")
        ax.legend()

        # Punto iniziale per l'animazione
        point, = ax.plot([], [], 'ro', label="Posizione Attuale")
        ax.legend()

        # Funzione di inizializzazione
        def init():
            point.set_data([], [])
            return point,

        # Funzione di animazione
        def animate(i):
            point.set_data(x_interpolated[i], y_interpolated[i])
            return point,

        # Creazione dell'animazione
        ani = FuncAnimation(fig, animate, frames=len(x_interpolated), init_func=init, blit=True, interval=25)

        plt.grid(True)
        plt.show()








# Test della libreria
if __name__ == "__main__":
    # Parametri di esempio
    generator = ProfileGenerator(2.0,0.5,2.0,0.5,38,1000)
    VelX,AccX,VelY,AccY,TjX,TjY = generator.SyncCoreXYAxis(0,0,200,120, Graph=True)
    generator.TrajectorySimulator2D(TjX,TjY)
    print("Speed X Axis: ", VelX)
    print("Acc/Dec X Axis: ", AccX)
    print("Speed Y Axis: ", VelY)
    print("Acc/Dec Y Axis: ", AccY)

