import openai
import json
import os

def generate_response(user_input: str) -> str:
    client = openai.OpenAI(api_key="sk-proj-d5M3eeV1P0wAYBU2eoQpT3BlbkFJkbaBKB2jFv1QRCq3ya9E")
    prompt_hci: str = """"
    文章から以下３つの情報を抽出してください。

    ・「こっち」「あっち」といった指示語や、「オレンジ」や「みかん」といった対象のオブジェクト名、「取ってきて」といったレトリーバルに関係した否定形でない命令の３つが入力に含まれているかのbool値(isOrder)
    ・対象のオブジェクト名のstring(object)
    ・「了解しました!取ってきます!」のようなユーザーへの返答のstring(response)
    以下の点に注意してください。
    ・出力はjson形式で１つお願いします。
    ・出力に"json"や""といったものは不要です。"{"から始まり"}"で終わるようにしてください。

    {
      "isOrder": bool,
      "object": string,
      "response": string
    }

    例：
    入力
    「あっちからみかん取ってきて」

    出力
    {
      "isOrder": true,
      "object": "みかん",
      "response": "了解しました!取ってきます!"
    }
    入力
    「あっちからみかん取ってこないで」

    出力
    {
      "isOrder": false,
      "object": "",
      "response": ""
    }
    入力
    「こっちにオレンジ持ってきて」

    出力
    {
      "isOrder": true,
      "object": "オレンジ",
      "response": "もちろんです!任せて下さい!"
    }

    入力
    「りんごを取ってきてください」

    出力
    {
      "isOrder": true,
      "object": "りんご",
      "response": "わあい！いいよ！"
    }

    入力
    「今日はいい天気ですね」

    出力
    {
      "isOrder": false,
      "object": "",
      "response": ""
    }

    <入力文>
    """
    try:
      completion = client.chat.completions.create(
        model="gpt-4o",
        messages=[
          {"role": "system", "content": prompt_hci},
          {"role": "user", "content": user_input} 
        ]
      )
    except Exception as e:
        raise e
    if completion.choices[0].message.content:
        return process_order(completion.choices[0].message.content)
    else:
        return process_order("")

def process_order(json_string: str):
    # JSON文字列を辞書に変換
    data = json.loads(json_string)
    # outputを作成
    output = {
        "isOrder": data["isOrder"],
        "object": data["object"],
        "response": data["response"]
    }
    return output


if __name__=="__main__":
    print(generate_response("あっちからりんご取ってきて"))
