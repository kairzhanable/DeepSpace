using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Motion Blur During Runtime.")]
	public class PostProcessingMotionBlur : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;
		
		[Tooltip("Activate or deactivate Motion Blur Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Motion Blur Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate Shutter Angle parameter")]
		public FsmBool ShutterAngleActive;
		[HasFloatSlider(0f, 360f)]
		[Tooltip("Scale Value")]
		public FsmFloat ShutterAngle;

		[Tooltip("Activate Sample Count parameter")]
		public FsmBool SampleActive;
		[HasFloatSlider(4, 32)]
		[Tooltip("Sample Count")]
		public FsmFloat SampleFloat;
		public FsmInt Sample;

		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private MotionBlur MotionBlurLayer;
		private PostProcessProfile UsedProfile;		
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			ShutterAngleActive = true;
			ShutterAngle = 270f;
			SampleActive = true;
			Sample = 4;
			everyFrame = false;
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetMotionBlur();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetMotionBlur();
		}

		void SetMotionBlur()
		{
						
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out MotionBlurLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<MotionBlur>())
				{
					if(MotionBlurLayer!=null)
					{
						MotionBlurLayer.active = ActiveLayer.Value; //Active Motion Blur Layer
						MotionBlurLayer.enabled.value = ActiveEffect.Value; // Active Motion Blur Effect
						MotionBlurLayer.shutterAngle.overrideState = ShutterAngleActive.Value;//Activate shutter angle
						if(ShutterAngleActive.Value)
						{ 	
							MotionBlurLayer.shutterAngle.value = ShutterAngle.Value; // shutter angle
						}
						MotionBlurLayer.sampleCount.overrideState = SampleActive.Value;//Activate sample count parameter
						if(SampleActive.Value)
						{ 	
							Sample.Value = Mathf.RoundToInt(SampleFloat.Value);
							MotionBlurLayer.sampleCount.value = Sample.Value; // Sample quantity
						}
					}
				}
			}
		}
}

