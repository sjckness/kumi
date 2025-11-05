#!/usr/bin/env python3
import os
import sys
import time
import numpy as np
import pygad

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor

from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import SetParameters

# ============================
# CONFIGURAZIONE — EDITA QUI
# ============================
NUM_SERVOS = 4
JOINT_NAMES = ['front_sh', 'front_ank', 'back_sh', 'back_ank']  # ordine desiderato
PID_NODE_NAME = "/PID_effort_controller"   # <-- nome del tuo nodo controller (es. dal launch)

# target angolare da raggiungere (deg) — puoi cambiarlo liberamente
ANGLE_TARGET = np.array([0, 90, 0, 30], dtype=float)

# tempi
SIM_TIME = 5.0     # secondi di valutazione per individuo
WARMUP = 0.5       # piccolo warmup dopo set target (s)
DT_HINT = 0.01     # solo per integrazione effort (se frequenza ≈100 Hz)

# pesi fitness (adatta alla tua dinamica)
W_POS = 1.0        # peso precisione (RMSE angolare)
W_EFF = 0.02       # peso effort integrato |tau|
W_OSC = 0.3        # peso oscillazione (varianza angolare finale)

# limiti genetici (range ragionevoli per servo)
KP_RANGE = (0.0, 20.0)
KI_RANGE = (0.0, 5.0)
KD_RANGE = (0.0, 5.0)

# ============================
# NODO ROS — UTILITIES
# ============================
class GAOrchestrator(Node):
    def __init__(self):
        super().__init__("ga_orchestrator")

        # target position publisher
        self.target_pub = self.create_publisher(Float64MultiArray, "/target_positions", 10)

        # world reset
        self.reset_cli = self.create_client(Empty, "/gazebo/reset_simulation")

        # pid param set
        self.param_cli = self.create_client(SetParameters, f"{PID_NODE_NAME}/set_parameters")

        # Subscriber joint states
        self.positions_log = []
        self.efforts_log = []
        self.idx_map_ready = False
        self.name_index = {}  # mappa name->index dentro i JointState

        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)

    # -----------------------------------------
    # CALLBACK: legge effort e position
    # -----------------------------------------
    def _joint_cb(self, msg: JointState):
        # La prima volta costruiamo la mappa name->index
        if not self.idx_map_ready:
            present = {n: i for i, n in enumerate(msg.name)}
            missing = [n for n in JOINT_NAMES if n not in present]
            if missing:
                # Non abbiamo ancora tutti i giunti: usciamo finché non compaiono
                self.get_logger().warn(f"Joint(s) non ancora presenti in /joint_states: {missing}")
                return
            self.name_index = present
            self.idx_map_ready = True
            self.get_logger().info(f"Indice joint pronto: {self.name_index}")

        # Estrai posizioni e sforzi nello stesso ordine JOINT_NAMES
        pos = []
        eff = []
        for jn in JOINT_NAMES:
            jidx = self.name_index[jn]
            # proteggi da array più corti
            p = msg.position[jidx] if jidx < len(msg.position) else 0.0
            e = msg.effort[jidx]   if jidx < len(msg.effort) else 0.0
            pos.append(p)
            eff.append(e)
        self.positions_log.append(pos)
        self.efforts_log.append(eff)

    # world reset
    def reset_sim(self, timeout=5.0):
        if not self.reset_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Servizio /gazebo/reset_simulation non disponibile.")
            return False
        req = Empty.Request()
        fut = self.reset_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = fut.result() is not None
        if ok:
            self.get_logger().info("🔄 Reset simulazione OK.")
        else:
            self.get_logger().error("Reset simulazione fallito/timeout.")
        return ok


    # pubish target position

    def publish_target(self, target: np.ndarray):
        msg = Float64MultiArray()
        msg.data = target.tolist()
        self.target_pub.publish(msg)

    # set PID params
    def set_pid_params(self, gains_flat):

        if not self.param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Servizio parametri non disponibile su {PID_NODE_NAME}.")
            return False

        # Costruisci la richiesta SetParameters
        from rcl_interfaces.msg import Parameter as ParamMsg
        from rcl_interfaces.msg import ParameterValue
        from rcl_interfaces.msg import ParameterType

        params_msgs = []
        for i in range(NUM_SERVOS):
            kp, ki, kd = gains_flat[i*3:(i+1)*3]
            base = f"{JOINT_NAMES[i]}"
            for name, val in (("Kp", kp), ("Ki", ki), ("Kd", kd)):
                pm = ParamMsg()
                pm.name = f"{base}.{name}"
                pm.value = Parameter(
                    name=pm.name,
                    value=val,
                    type_=Parameter.Type.DOUBLE
                ).to_parameter_msg().value  # riusa conversione rclpy
                params_msgs.append(pm)

        req = SetParameters.Request()
        req.parameters = params_msgs

        fut = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error("SetParameters fallito (timeout?).")
            return False

        # Check resulti (opzionale)
        return True

    # raccolta dati (x secondi)
    def collect_for(self, duration_s: float):
        self.positions_log = []
        self.efforts_log = []
        t0 = time.time()
        while (time.time() - t0) < duration_s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
        # ritorna np.array shape [N, NUM_SERVOS]
        return (np.asarray(self.positions_log, dtype=float),
                np.asarray(self.efforts_log, dtype=float))


# ============================
# FITNESS
# ============================
def compute_fitness(joint_angles: np.ndarray,
                    efforts: np.ndarray,
                    angle_target: np.ndarray,
                    dt_hint: float) -> float:

    if joint_angles.size == 0:
        return 0.0

    # 1) Precisione (RMSE media tra angolo e target)
    err = joint_angles - angle_target[None, :]
    rmse_per_joint = np.sqrt(np.mean(err**2, axis=0))
    mean_rmse = float(np.mean(rmse_per_joint))
    f_pos = 1.0 / (1.0 + mean_rmse)  # ∈ (0,1]

    # 2) Effort totale (proxy energia)
    total_effort = float(np.sum(np.abs(efforts))) * float(dt_hint)

    # 3) Oscillazione: varianza negli ultimi 20% campioni
    n = joint_angles.shape[0]
    tail = joint_angles[int(0.8 * n):, :] if n >= 10 else joint_angles
    osc = float(np.mean(np.var(tail, axis=0))) if tail.size else 0.0

    fitness = W_POS * f_pos - W_EFF * total_effort - W_OSC * osc
    return max(0.0, fitness)


# ============================
# PYGAD — FITNESS WRAPPER
# ============================
# NB: PyGAD 2.20.0 richiede 3 argomenti (ga_instance, solution, solution_idx)
def make_fitness_fn(node: GAOrchestrator):
    def fitness_fn(ga_instance, solution, solution_idx):
        # Stampa di servizio per capire a che punto siamo
        gen = ga_instance.generations_completed + 1
        print(f"\n[Gen {gen}/{ga_instance.num_generations}] individuo {solution_idx+1}/{ga_instance.sol_per_pop}")

        # reset simulazione
        node.reset_sim()
            
        # 2) set PID
        node.set_pid_params(solution)
            
        # 3) reset target → 0 rad (riporta i servo a home)
        node.publish_target(np.zeros(NUM_SERVOS, dtype=float))
        # piccolo warmup per far valutare il reset + raggiungere 0
        node.collect_for(WARMUP)

        # 4) imposta target desiderato
        node.publish_target(ANGLE_TARGET)

        # 5) raccogli dati per SIM_TIME (posizioni + effort)
        #    nota: qui NON imponiamo un dt fisso; usiamo dt_hint per integrare effort
        joint_angles, efforts = node.collect_for(SIM_TIME)

        # 6) calcolo fitness
        fit = compute_fitness(joint_angles, efforts, ANGLE_TARGET, DT_HINT)
        print(f"fitness = {fit:.4f}")
        return fit
    return fitness_fn


# ============================
# MAIN — GA SETUP & RUN
# ============================
def main():
    # 0) avvia ROS (Gazebo + controller devono essere già in esecuzione dal launch)
    rclpy.init()
    node = GAOrchestrator()

    # 1) definisci spazio dei geni: [Kp1, Ki1, Kd1, ..., Kp4, Ki4, Kd4]
    gene_space = []
    for _ in range(NUM_SERVOS):
        gene_space.extend([
            {"low": KP_RANGE[0], "high": KP_RANGE[1]},  # Kp
            {"low": KI_RANGE[0], "high": KI_RANGE[1]},  # Ki
            {"low": KD_RANGE[0], "high": KD_RANGE[1]},  # Kd
        ])

    # 2) crea fitness function che chiama ROS
    fitness_func = make_fitness_fn(node)

    # 3) istanzia GA
    ga = pygad.GA(
        num_generations=10,        # aumenta quando tutto gira stabile
        sol_per_pop=12,
        num_parents_mating=6,
        num_genes=NUM_SERVOS * 3,
        fitness_func=fitness_func,
        gene_space=gene_space,
        mutation_percent_genes=25,
        crossover_type="single_point",
        mutation_type="random",
        keep_elitism=2,
        allow_duplicate_genes=False,
        # parallel_processing: meglio evitare qui, usiamo una sola istanza Gazebo condivisa
    )

    print("Avvio GA… ")
    ga.run()

    best_sol, best_fit, _ = ga.best_solution()
    print("\n Miglior soluzione trovata:")
    for i in range(NUM_SERVOS):
        Kp, Ki, Kd = best_sol[i*3:(i+1)*3]
        print(f"  {JOINT_NAMES[i]} → Kp={Kp:.3f}  Ki={Ki:.3f}  Kd={Kd:.3f}")
    print(f"Best fitness = {best_fit:.4f}")

    # chiudi ROS
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
