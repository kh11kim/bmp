import pandas as pd
from datetime import datetime

def save(result, test_name, save_dir="./test_result/"):
    now = datetime.now()
    time_str = now.strftime("%y%m%d_%H%M%S")
    filename = f"{save_dir}{time_str}_{test_name}.csv"
    df = pd.DataFrame()
    df[test_name] = result
    df.to_csv(filename, index=False)
    print(f"saved. test:{test_name}")


if __name__ == "__main__":
    result = [1,2,3,4,5,6]
    save(result, "test")