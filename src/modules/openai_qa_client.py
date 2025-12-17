import os
import json
import requests

class OpenAIQAClient(object):
    def __init__(self, model="gpt-5-mini", timeout_s=20):
        self.model = model
        self.timeout_s = timeout_s

    def ask(self, question, instructions=None, max_output_tokens=500):
        api_key = os.environ.get("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY is not set")

        payload = {
            "model": self.model,
            "input": question,
            "max_output_tokens": max_output_tokens
        }
        if instructions:
            payload["instructions"] = instructions

        headers = {
            "Authorization": "Bearer %s" % api_key,
            "Content-Type": "application/json"
        }

        r = requests.post(
            "https://api.openai.com/v1/responses",
            headers=headers,
            data=json.dumps(payload),
            timeout=self.timeout_s
        )
        r.raise_for_status()
        data = r.json()

        # Collect text parts from the Responses API output structure
        # --- Extract text robustly (avoid "[No text output]") ---

        # 1) Some responses include a top-level output_text
        if data.get("output_text"):
            return data["output_text"].strip()

        texts = []

        # 2) Most common: structured output list
        for item in data.get("output", []):
            itype = item.get("type")

            # Some SDKs return top-level output_text items
            if itype in ("output_text", "text") and item.get("text"):
                texts.append(item["text"])

            # Normal: message -> content -> output_text/text
            if itype == "message":
                for part in item.get("content", []):
                    ptype = part.get("type")
                    if ptype in ("output_text", "text") and part.get("text"):
                        texts.append(part["text"])
                    if ptype == "refusal" and part.get("refusal"):
                        texts.append(part["refusal"])

        if texts:
            return ("\n".join(texts)).strip()

        # 3) Last resort: recursively grab any "text"/"output_text" strings anywhere
        def _collect(obj, out):
            try:
                basestring  # py2
            except NameError:
                basestring = str
            if isinstance(obj, dict):
                for k, v in obj.items():
                    if k in ("text", "output_text") and isinstance(v, basestring):
                        out.append(v)
                    else:
                        _collect(v, out)
            elif isinstance(obj, list):
                for v in obj:
                    _collect(v, out)

        out = []
        _collect(data, out)
        out = [s.strip() for s in out if s and s.strip()]
        return out[0] if out else "[No text output]"

