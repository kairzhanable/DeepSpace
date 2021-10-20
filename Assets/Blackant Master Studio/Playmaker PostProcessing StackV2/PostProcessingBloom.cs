using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Bloom During Runtime.")]
	public class PostProcessingBloom : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Bloom Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Bloom Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate or Intensity Parameter")]
		public FsmBool IntensityActive;
		[HasFloatSlider(0f, 100f)]
		[Tooltip("Bloom Intensity")]
		public FsmFloat Intensity;
		[Tooltip("Activate or deactivate Threshold")]
		public FsmBool ThresholdActive;
		[HasFloatSlider(0f, 100f)]
		[Tooltip("Bloom Threshold")]
		public FsmFloat Threshold;
		[Tooltip("Activate or deactivate SoftKnee Parameter")]
		public FsmBool SoftKneeActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("Bloom SoftKnee")]
		public FsmFloat SoftKnee;
		[Tooltip("Activate or deactivate Clamp Parameter")]
		public FsmBool ClampActive;
		[Tooltip("Clamp value")]
		public FsmFloat Clamp;
		[Tooltip("Activate or deactivate Diffusion Parameter")]
		public FsmBool DiffusionActive;
		[HasFloatSlider(1f, 10f)]
		[Tooltip("Bloom Diffusion value")]
		public FsmFloat Diffusion;
		[Tooltip("Activate or deactivate Anamorphic parameter")]
		public FsmBool AnamorphicRatioActive;
		[HasFloatSlider(-1f, 1f)]
		[Tooltip("Anamophic Value")]
		public FsmFloat AnamorphicRatio;
		[Tooltip("Activate or deactivate Color parameter")]
		public FsmBool ColorActive;
		[Tooltip("Bloom Color")]
		public FsmColor BloomColor;
		[Tooltip("Activate or deactivate Fast mode")]
		public FsmBool FastModeActive;
		[Tooltip("Set Fast Mode")]
		public FsmBool FastMode;

		[Tooltip("Activate or deactivate DirtTexture parameter")]
		public FsmBool DirtTextureActive;
		[Tooltip("Set Dirt Texture")]
		public FsmTexture DirtTexture;
		[Tooltip("Activate or deactivate Dirt Texture intensity")]
		public FsmBool DirtIntensityActive;
		[HasFloatSlider(0f, 100f)]
		[Tooltip("Dirt Texture Intensity")]
		public FsmFloat DirtIntensity;

		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion
		
		#region Private variables
		private Bloom BloomLayer;
		private PostProcessProfile UsedProfile; 		
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			IntensityActive= true;
			Intensity=0f;
			ThresholdActive= true;
			Threshold=1f;
			SoftKneeActive= true;
			SoftKnee=0.5f;
			ClampActive= true;
			Clamp=65472;
			DiffusionActive= true;
			Diffusion=7f;
			AnamorphicRatioActive= true;
			AnamorphicRatio=0f;
			ColorActive= true;
			BloomColor = Color.white;
			FastModeActive= true;
			FastMode= true;
			DirtTextureActive= true;
			DirtTexture = null;
			DirtIntensityActive= true;
			DirtIntensity = 0f;
			everyFrame = false;
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetBloomParameteres();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetBloomParameteres();
		}

		void SetBloomParameteres()
		{
							
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out BloomLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<Bloom>())
				{
					if(BloomLayer!=null)
					{
						BloomLayer.active = ActiveLayer.Value; //Active Bloom Layer
						BloomLayer.enabled.value = ActiveEffect.Value; // Active Bloom Effect
						BloomLayer.intensity.overrideState = IntensityActive.Value ;//Activate intensity
						if(IntensityActive.Value)
						{
							BloomLayer.intensity.value = Intensity.Value ; // set intensity
						}
						BloomLayer.threshold.overrideState = ThresholdActive.Value ;//Activate Threshold
						if(ThresholdActive.Value)
						{
							BloomLayer.threshold.value = Threshold.Value ; // set Threshold
						}
						BloomLayer.softKnee.overrideState = SoftKneeActive.Value ;//Activate SoftKnee
						if(SoftKneeActive.Value)
						{
							BloomLayer.softKnee.value = SoftKnee.Value ; // set SoftKnee
						}
						BloomLayer.clamp.overrideState = ClampActive.Value ;//Activate Clamp
						if(ClampActive.Value)
						{
							BloomLayer.clamp.value = Clamp.Value ; // set Clamp value
						}
						BloomLayer.diffusion.overrideState = DiffusionActive.Value ;//Activate Diffusion
						if(DiffusionActive.Value)
						{
							BloomLayer.diffusion.value = Diffusion.Value ; // set Diffusion value
						}
						BloomLayer.anamorphicRatio.overrideState = AnamorphicRatioActive.Value ;//Activate Anamorphic
						if(AnamorphicRatioActive.Value)
						{
							BloomLayer.anamorphicRatio.value = AnamorphicRatio.Value ; // set Anamorphic value
						}
						BloomLayer.color.overrideState = ColorActive.Value ;//Activate Color
						if(ColorActive.Value)
						{
							BloomLayer.color.value = BloomColor.Value ; // set Color value
						}
						BloomLayer.fastMode.overrideState = FastModeActive.Value ;//Activate FastMode
						if(FastModeActive.Value)
						{
							BloomLayer.fastMode.value = FastMode.Value ; // set Fastmode
						}
						BloomLayer.dirtTexture.overrideState = DirtTextureActive.Value ;//Activate Dirt Texture
						if(DirtTextureActive.Value)
						{
							BloomLayer.dirtTexture.value = DirtTexture.Value ; // set Dirt Texture
						}
						BloomLayer.dirtIntensity.overrideState = DirtIntensityActive.Value ;//Activate Dirt Texture Intensity
						if(DirtIntensityActive.Value)
						{
							BloomLayer.dirtIntensity.value = DirtIntensity.Value ; // set dirt Texture Intensity
						}
				}
			}
		}
	}

}


