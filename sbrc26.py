import os
import threading
import pandas as pd
from concurrent.futures import ProcessPoolExecutor
import time
from datetime import datetime

result_path = '/home/thiago/Doutorado/SAS26/results'
ns3_path = '/home/thiago/ns-3-allinone/ns-3.44'
ns3_cmd = f'{ns3_path}/./ns3'
script = 'scratch/sbrc26/sbrc26.cc'

def run_simulation(ns3_cmd, script, params01, params02, params3, j):
    timestamp_start = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp_start}] [INFO] Iniciando execução: run {j}")

    start_time = time.time()
    run_cmd = f'{ns3_cmd} run "{script} {params01} {params02} {params3} --nRun={j}"'
    exit_code = os.system(run_cmd)
    end_time = time.time()

    duration = round(end_time - start_time, 2)
    timestamp_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp_end}] [INFO] Finalizado run {j} | Código: {exit_code} | Duração: {duration}s")

    return duration

def simulate(script, path, sm_coords, gw_coords, radius, sfa, ns3_cmd,
             adr_enabled=0, adr_type="ns3::AdrComponent", adr_name="adr"):
    init_iters = [1]
    end_iters = [2]

    # Lista para armazenar duração de cada simulação
    durations = []

    # Detectar número de núcleos
    max_procs = os.cpu_count()
    print(f"[INFO] Detectado {max_procs} núcleos. É possível executar até {max_procs} simulações em paralelo.")

    # Salvar arquivos CSV
    sm_file = make_file_name(path, f'{len(sm_coords)}sm_file')
    pd.DataFrame(sm_coords).to_csv(sm_file, header=False, index=False)
    gw_file = make_file_name(path, f'{len(gw_coords)}gw_file')
    pd.DataFrame(gw_coords).to_csv(gw_file, header=False, index=False)

    # Parâmetros fixos
    params01 = f'--nDevices={len(sm_coords)} --nGateways={len(gw_coords)} --path={path}'
    params02 = f'--smFile={sm_file} --gwFile={gw_file} --radius={radius} --sfa={sfa}'
    params03 = f'--adrEnabled={adr_enabled} --adrType={adr_type} --adrName={adr_name}'

    # Rodar comando inicial do ns3 (se necessário)
    os.system(ns3_cmd)

    # Executor com processos
    with ProcessPoolExecutor(max_workers=max_procs) as executor:
        futures = []
        for i in range(len(init_iters)):
            for j in range(init_iters[i], end_iters[i]):
                futures.append(executor.submit(run_simulation, ns3_cmd, script, 
                                               params01, params02, params03, j))

        # Capturar a duração de cada simulação
        for future in futures:
            durations.append(future.result())

    print(f"\n[INFO] Durações de todas as simulações: {durations}")
    print('#'*100)
    return durations

'''def simulate_sfa(script, nDevices, nGateways, sfa, path, sm_coords, gw_coords, radius, 
                 tx_mode="nack", adr_enabled=0, adr_type="ns3::AdrComponent", adr_name="adr"):
  init_iters = [1]
  end_iters = [11]
  threads = []

  sm_file = make_file_name(path, f'{len(sm_coords)}sm_file')
  pd.DataFrame(sm_coords).to_csv(sm_file, header=False, index=False)
  gw_file = make_file_name(path, f'{len(gw_coords)}gw_file')
  pd.DataFrame(gw_coords).to_csv(gw_file, header=False, index=False)

  params01 = f'--nDevices={nDevices} --nGateways={nGateways} --sfa={sfa} --path={path} --radius={radius}'
  params02 = f'--smFile={sm_file} --gwFile={gw_file} --txMode={tx_mode} --adrEnabled={adr_enabled}'
  params03 = f'--adrType={adr_type} --adrName={adr_name}'

  for i in range(len(init_iters)):
    os.system(ns3_cmd)
    for j in range(init_iters[i], end_iters[i]):
      run_cmd = f'{ns3_cmd} run "{script} {params01} {params02} {params03} --nRun={j}"'  # --gdb
      t = threading.Thread(target=os.system, args=[run_cmd])
      threads.append(t)
      t.start()

    for t in threads:
      t.join()

  os.system(ns3_cmd)'''

def make_file_name(path, name, ext='csv'):
   return f'{path}/{name}.{ext}'

def check(file):
   names = [
        'sent', 'rec', 'pdr', 'imr_sent', 'imr_rec', 'imr_pdr', 'an_sent', 'an_rec',
        'an_pdr', 'delay', 'imr_delay', 'pcc_delay', 'rssi', 'snr', 'energy', 'tput',
        'ee1', 'ee2', 'ee3', 'ee4', 'rssi_pkts', 'snr_pkts', 'nRun'
   ]

   df = pd.read_csv(file, names=names)
   print(f"IMR Min. PDR = {df['imr_pdr'].min()} and AN Min. PDR = {df['an_pdr'].min()}")

   return df['imr_pdr'].min() >= 99 and df['an_pdr'].min() >= 99

def test_isfa():
  sfa = 'isfa'
  scenarios = [200, 400, 600, 800, 1000]
  coords_dir = '/home/thiago/Doutorado/SAS26/sas26'
  radius = 7000

  # ISFA
  k = 1
  for _, scenario in enumerate(scenarios):
     print(f'{scenario} SMs')
     path = f'{result_path}/{sfa}/{scenario}'
     
     sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
     gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
     simulate(script, path, sm_coords, gw_coords, radius, sfa, ns3_cmd)
     filesfa = make_file_name(path, f'{k}gw_data')
     
     while(check(filesfa) == False):
        k = k + 1
        if(k == 29):
           k = 28
           break
        
        sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
        gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
        simulate(script, path, sm_coords, gw_coords, radius, sfa, ns3_cmd)
        filesfa = make_file_name(path, f'{k}gw_data')

     pd.DataFrame([k]).to_csv(make_file_name(path, 'k'), header=False, index=False)

def test_adr():
  sfa = 'none'
  scenarios = [200, 400, 600, 800, 1000]
  radius = 7000
  coords_dir = '/home/thiago/Doutorado/SAS26/sas26'

  # ADR
  k = 1
  for i, scenario in enumerate(scenarios):
     print(f'{scenario} SMs')
     path = f'{result_path}/adr/{scenario}'
     
     sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
     gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
     simulate(script, path, sm_coords, gw_coords, radius, 
              sfa, ns3_cmd, adr_enabled=1, adr_name='adr')
     filesfa = make_file_name(path, f'{k}gw_data')
     
     while(check(filesfa) == False):
        k = k + 1
        if(k == 29):
           k = 28
           break
        
        sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
        gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
        simulate(script, path, sm_coords, gw_coords, radius, 
                 sfa, ns3_cmd, adr_enabled=1, adr_name='adr')
        filesfa = make_file_name(path, f'{k}gw_data')

     pd.DataFrame([k]).to_csv(make_file_name(path, 'k'), header=False, index=False)

def test_caadr():
  sfa = 'none'
  scenarios = [200, 400, 600, 800, 1000]
  radius = 7000
  coords_dir = '/home/thiago/Doutorado/SAS26/sas26'

  # CA-ADR
  k = 1
  for i, scenario in enumerate(scenarios):
     print(f'{scenario} SMs')
     path = f'{result_path}/caadr/{scenario}'
     
     sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
     gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
     simulate(script, path, sm_coords, gw_coords, radius, 
              sfa, ns3_cmd, adr_enabled=1, adr_type='ns3::CAADR', adr_name='caadr')
     filesfa = make_file_name(path, f'{k}gw_data')
     
     while(check(filesfa) == False):
        k = k + 1
        if(k == 29):
           k = 28
           break
        
        sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
        gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
        simulate(script, path, sm_coords, gw_coords, radius, 
                 sfa, ns3_cmd, adr_enabled=1, adr_type='ns3::CAADR', adr_name='caadr')
        filesfa = make_file_name(path, f'{k}gw_data')

     pd.DataFrame([k]).to_csv(make_file_name(path, 'k'), header=False, index=False)

def test_mbadr():
  sfa = 'none'
  scenarios = [200, 400, 600, 800, 1000]
  radius = 7000
  coords_dir = '/home/thiago/Doutorado/SAS26/sas26'

  # MB-ADR
  k = 1
  for i, scenario in enumerate(scenarios):
     print(f'{scenario} SMs')
     path = f'{result_path}/mbadr/{scenario}'
     
     sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
     gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
     simulate(script, path, sm_coords, gw_coords, radius, 
              sfa, ns3_cmd, adr_enabled=1, adr_type='ns3::MBADR', adr_name='mbadr')
     filesfa = make_file_name(path, f'{k}gw_data')
     
     while(check(filesfa) == False):
        k = k + 1
        if(k == 29):
           k = 28
           break
        
        sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
        gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
        simulate(script, path, sm_coords, gw_coords, radius, 
                 sfa, ns3_cmd, adr_enabled=1, adr_type='ns3::MBADR', adr_name='mbadr')
        filesfa = make_file_name(path, f'{k}gw_data')

     pd.DataFrame([k]).to_csv(make_file_name(path, 'k'), header=False, index=False)

def test_drsfa():
  sfa = 'drsfa'
  scenarios = [200, 400, 600, 800, 1000]
  radius = 7000
  coords_dir = '/home/thiago/Doutorado/SAS26/sas26'

  # DR-SFA
  k = 1
  for i, scenario in enumerate(scenarios):
     print(f'{scenario} SMs')
     path = f'{result_path}/{sfa}/{scenario}'
     
     sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
     gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
     simulate(script, path, sm_coords, gw_coords, radius, 
              sfa, ns3_cmd)
     filesfa = make_file_name(path, f'{k}gw_data')
     
     while(check(filesfa) == False):
        k = k + 1
        if(k == 29):
           k = 28
           break
        
        sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
        gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
        simulate(script, path, sm_coords, gw_coords, radius, 
                 sfa, ns3_cmd)
        filesfa = make_file_name(path, f'{k}gw_data')

     pd.DataFrame([k]).to_csv(make_file_name(path, 'k'), header=False, index=False)

def test_drsftpa():
  sfa = 'drsftpa'
  scenarios = [200, 400, 600, 800, 1000]
  radius = 7000
  coords_dir = '/home/thiago/Doutorado/SAS26/sas26'

  # DR-SFTPA
  k = 1
  for i, scenario in enumerate(scenarios):
     print(f'{scenario} SMs')
     path = f'{result_path}/{sfa}/{scenario}'
     
     sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
     gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
     simulate(script, path, sm_coords, gw_coords, radius, 
              sfa, ns3_cmd)
     filesfa = make_file_name(path, f'{k}gw_data')
     
     while(check(filesfa) == False):
        k = k + 1
        if(k == 29):
           k = 28
           break
        
        sm_coords = pd.read_csv(f'{coords_dir}/{scenario}/{scenario}sms.csv', names=['x', 'y'])[['x', 'y']].values
        gw_coords = pd.read_csv(f'{coords_dir}/{scenario}/{k}gws.csv', names=['x', 'y'])[['x', 'y']].values
        simulate(script, path, sm_coords, gw_coords, radius, 
                 sfa, ns3_cmd)
        filesfa = make_file_name(path, f'{k}gw_data')

     pd.DataFrame([k]).to_csv(make_file_name(path, 'k'), header=False, index=False)

if __name__ == "__main__":
    test_adr()
    test_isfa()
    test_caadr()
    test_drsfa()
    test_drsftpa()